#include "preview_server.hpp"

#include <arpa/inet.h>
#include <csignal>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <jpeglib.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace {

constexpr const char* BOUNDARY = "uminoidpreview";

constexpr const char* INDEX_HTML =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<title>RealSense preview</title>"
    "<style>html,body{margin:0;background:#111;height:100%}"
    "body{display:flex;align-items:center;justify-content:center}"
    "img{max-width:100vw;max-height:100vh;image-rendering:pixelated}"
    "</style></head><body>"
    "<img src=\"/stream.mjpg\" alt=\"preview\"></body></html>";

// Nearest-neighbour downscale; src is RGB8 with explicit stride, dst is tight.
void downscale_rgb(const uint8_t* src, int sw, int sh, int src_stride,
                   uint8_t* dst, int dw, int dh) {
    for (int y = 0; y < dh; ++y) {
        const int sy = (y * sh) / dh;
        const uint8_t* srow = src + sy * src_stride;
        uint8_t* drow = dst + y * dw * 3;
        for (int x = 0; x < dw; ++x) {
            const int sx = (x * sw) / dw;
            const uint8_t* p = srow + sx * 3;
            drow[x * 3 + 0] = p[0];
            drow[x * 3 + 1] = p[1];
            drow[x * 3 + 2] = p[2];
        }
    }
}

bool encode_jpeg(const uint8_t* rgb, int w, int h, int quality,
                 std::vector<uint8_t>& out) {
    jpeg_compress_struct cinfo{};
    jpeg_error_mgr       jerr{};
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    unsigned char* buf  = nullptr;
    unsigned long  size = 0;
    jpeg_mem_dest(&cinfo, &buf, &size);

    cinfo.image_width      = w;
    cinfo.image_height     = h;
    cinfo.input_components = 3;
    cinfo.in_color_space   = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row = const_cast<JSAMPROW>(rgb + cinfo.next_scanline * w * 3);
        jpeg_write_scanlines(&cinfo, &row, 1);
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    if (buf) {
        out.assign(buf, buf + size);
        free(buf);
        return true;
    }
    return false;
}

bool write_all(int fd, const void* data, size_t n) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (n > 0) {
        ssize_t w = ::send(fd, p, n, MSG_NOSIGNAL);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += w;
        n -= static_cast<size_t>(w);
    }
    return true;
}

}  // namespace

PreviewServer::PreviewServer(Config cfg,
                             std::function<void(const std::string&)> on_error)
    : cfg_(std::move(cfg)), on_error_(std::move(on_error)) {}

PreviewServer::~PreviewServer() { stop(); }

void PreviewServer::start() {
    if (running_.exchange(true)) return;
    // A dropped HTTP client will cause send() to fail gracefully instead of
    // killing the recorder process.
    std::signal(SIGPIPE, SIG_IGN);

    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        running_.store(false);
        on_error_("[Preview] socket() failed");
        return;
    }

    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(cfg_.port));
    if (::inet_pton(AF_INET, cfg_.bind_host.c_str(), &addr.sin_addr) != 1) {
        ::close(listen_fd_); listen_fd_ = -1; running_.store(false);
        on_error_("[Preview] bad bind host: " + cfg_.bind_host);
        return;
    }

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(listen_fd_); listen_fd_ = -1; running_.store(false);
        on_error_("[Preview] bind failed on " + cfg_.bind_host + ":" +
                  std::to_string(cfg_.port));
        return;
    }
    if (::listen(listen_fd_, 4) < 0) {
        ::close(listen_fd_); listen_fd_ = -1; running_.store(false);
        on_error_("[Preview] listen() failed");
        return;
    }

    accept_thread_ = std::thread([this] { accept_loop_(); });
    encode_thread_ = std::thread([this] { encode_loop_(); });
}

void PreviewServer::stop() {
    if (!running_.exchange(false)) return;
    stage_cv_.notify_all();
    jpeg_cv_.notify_all();
    if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (accept_thread_.joinable()) accept_thread_.join();
    if (encode_thread_.joinable()) encode_thread_.join();
}

void PreviewServer::push_imu(float pitch_deg, float roll_deg) {
    latest_pitch_deg_.store(pitch_deg, std::memory_order_relaxed);
    latest_roll_deg_.store(roll_deg,  std::memory_order_relaxed);
}

void PreviewServer::push_rgb(const uint8_t* rgb, int width, int height, int stride) {
    if (!running_.load()) return;
    const size_t need = static_cast<size_t>(stride) * static_cast<size_t>(height);
    std::lock_guard<std::mutex> lk(stage_mu_);
    if (staged_rgb_.size() < need) staged_rgb_.resize(need);
    std::memcpy(staged_rgb_.data(), rgb, need);
    staged_w_      = width;
    staged_h_      = height;
    staged_stride_ = stride;
    ++staged_seq_;
    stage_cv_.notify_one();
}

void PreviewServer::accept_loop_() {
    while (running_.load()) {
        sockaddr_in cli{};
        socklen_t   clilen = sizeof(cli);
        int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&cli), &clilen);
        if (fd < 0) {
            if (!running_.load()) return;
            continue;
        }
        std::thread([this, fd] { serve_client_(fd); }).detach();
    }
}

void PreviewServer::encode_loop_() {
    const auto min_interval =
        std::chrono::milliseconds(1000 / std::max(1, cfg_.max_fps));
    auto last_tick = std::chrono::steady_clock::now() - min_interval;

    std::vector<uint8_t> downscaled(
        static_cast<size_t>(cfg_.out_width) * cfg_.out_height * 3);
    std::vector<uint8_t> rgb_copy;

    while (running_.load()) {
        int sw = 0, sh = 0, sstride = 0;
        uint64_t seq = 0;
        {
            std::unique_lock<std::mutex> lk(stage_mu_);
            stage_cv_.wait(lk, [&] {
                return !running_.load() || staged_seq_ != consumed_seq_;
            });
            if (!running_.load()) return;

            sw      = staged_w_;
            sh      = staged_h_;
            sstride = staged_stride_;
            seq     = staged_seq_;
            rgb_copy.assign(staged_rgb_.begin(),
                            staged_rgb_.begin() + sstride * sh);
            consumed_seq_ = seq;
        }

        // Rate limit after we've consumed, so we never starve clients of the
        // very last frame.
        const auto now  = std::chrono::steady_clock::now();
        const auto wait = min_interval - (now - last_tick);
        if (wait.count() > 0) std::this_thread::sleep_for(wait);
        last_tick = std::chrono::steady_clock::now();

        downscale_rgb(rgb_copy.data(), sw, sh, sstride,
                      downscaled.data(), cfg_.out_width, cfg_.out_height);

        std::vector<uint8_t> jpeg;
        if (!encode_jpeg(downscaled.data(), cfg_.out_width, cfg_.out_height,
                         cfg_.jpeg_quality, jpeg)) {
            continue;
        }

        {
            std::lock_guard<std::mutex> jl(jpeg_mu_);
            latest_jpeg_ = std::move(jpeg);
            ++latest_jpeg_seq_;
        }
        jpeg_cv_.notify_all();
    }
}

void PreviewServer::serve_client_(int fd) {
    // Minimal HTTP: read the request headers, then route on the path.
    // We only need the request line; clients don't send bodies here.
    std::string req;
    char buf[1024];
    while (req.find("\r\n\r\n") == std::string::npos && req.size() < 8192) {
        ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) { ::close(fd); return; }
        req.append(buf, buf + n);
    }

    std::string path = "/";
    const auto sp1 = req.find(' ');
    const auto sp2 = sp1 == std::string::npos ? std::string::npos
                                              : req.find(' ', sp1 + 1);
    if (sp1 != std::string::npos && sp2 != std::string::npos) {
        path = req.substr(sp1 + 1, sp2 - sp1 - 1);
    }

    if (path == "/" || path == "/index.html") {
        std::ostringstream r;
        r << "HTTP/1.0 200 OK\r\n"
          << "Content-Type: text/html; charset=utf-8\r\n"
          << "Content-Length: " << std::strlen(INDEX_HTML) << "\r\n"
          << "Cache-Control: no-store\r\n"
          << "Connection: close\r\n\r\n"
          << INDEX_HTML;
        const std::string out = r.str();
        write_all(fd, out.data(), out.size());
        ::close(fd);
        return;
    }

    if (path == "/imu") {
        std::ostringstream body;
        body << "{\"pitch_deg\":" << std::fixed << std::setprecision(2)
             << latest_pitch_deg_.load(std::memory_order_relaxed)
             << ",\"roll_deg\":" << std::fixed << std::setprecision(2)
             << latest_roll_deg_.load(std::memory_order_relaxed) << "}";
        const std::string b = body.str();
        std::ostringstream r;
        r << "HTTP/1.0 200 OK\r\n"
          << "Content-Type: application/json\r\n"
          << "Content-Length: " << b.size() << "\r\n"
          << "Cache-Control: no-store\r\n"
          << "Connection: close\r\n\r\n"
          << b;
        const std::string out = r.str();
        write_all(fd, out.data(), out.size());
        ::close(fd);
        return;
    }

    if (path != "/stream.mjpg") {
        static const char* k404 =
            "HTTP/1.0 404 Not Found\r\nContent-Length: 0\r\n"
            "Connection: close\r\n\r\n";
        write_all(fd, k404, std::strlen(k404));
        ::close(fd);
        return;
    }

    // MJPEG stream. Browsers render multipart/x-mixed-replace inline.
    std::ostringstream header;
    header << "HTTP/1.0 200 OK\r\n"
           << "Cache-Control: no-store\r\n"
           << "Pragma: no-cache\r\n"
           << "Connection: close\r\n"
           << "Content-Type: multipart/x-mixed-replace; boundary="
           << BOUNDARY << "\r\n\r\n";
    const std::string hdr = header.str();
    if (!write_all(fd, hdr.data(), hdr.size())) { ::close(fd); return; }

    uint64_t last_seq = 0;
    while (running_.load()) {
        std::vector<uint8_t> jpeg;
        {
            std::unique_lock<std::mutex> lk(jpeg_mu_);
            jpeg_cv_.wait(lk, [&] {
                return !running_.load() || latest_jpeg_seq_ != last_seq;
            });
            if (!running_.load()) break;
            last_seq = latest_jpeg_seq_;
            jpeg     = latest_jpeg_;
        }

        std::ostringstream part;
        part << "--" << BOUNDARY << "\r\n"
             << "Content-Type: image/jpeg\r\n"
             << "Content-Length: " << jpeg.size() << "\r\n\r\n";
        const std::string phdr = part.str();
        if (!write_all(fd, phdr.data(), phdr.size()))    break;
        if (!write_all(fd, jpeg.data(), jpeg.size()))    break;
        if (!write_all(fd, "\r\n", 2))                   break;
    }
    ::close(fd);
}
