#include "zmq_camera_recorder.hpp"

using namespace camera_frame;
using namespace zmq_camera_constants;

#include "camera/preview_server.hpp"
#include "utils/repo_constants.hpp"
#include "utils/time.hpp"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

ZmqCameraRecorder::ZmqCameraRecorder(const std::string& recording_label,
                                     const std::function<void(const std::string&)>& raise_error,
                                     PreviewServer* preview)
    : output_dir_(repo_constants::DATA_DIR + "/" + recording_label),
      raise_error_(raise_error),
      csv_(output_dir_ + "/camera.csv",
           "collection_id,frame_number,camera_timestamp_ms,host_timestamp,pitch_deg,roll_deg"),
      ctx_(1),
      sock_(ctx_, zmq::socket_type::pull),
      preview_(preview) {
    try {
        std::filesystem::create_directories(output_dir_ + "/frames");
        sock_.set(zmq::sockopt::conflate, 1);
        sock_.set(zmq::sockopt::linger, 0);
        sock_.connect(ENDPOINT);
    } catch (const std::exception& e) {
        raise_error_(std::string("[ZmqCamera] ") + e.what());
    }
}

bool ZmqCameraRecorder::write_bytes(const std::string& filename,
                                    const uint8_t* data, size_t size) const {
    std::ofstream out(filename, std::ios::binary);
    if (!out) return false;
    out.write(reinterpret_cast<const char*>(data),
              static_cast<std::streamsize>(size));
    return static_cast<bool>(out);
}

void ZmqCameraRecorder::collect_loop(const std::function<int()>&  collection_id,
                                     const std::function<bool()>& stop,
                                     const std::function<bool()>& pause) {
    try {
        const std::string frames_dir = output_dir_ + "/frames";
        zmq::pollitem_t item[] = { { sock_.handle(), 0, ZMQ_POLLIN, 0 } };
        const std::chrono::milliseconds stale_after(RECV_TIMEOUT_MS);

        int frame_count = 0;
        auto last_packet = std::chrono::steady_clock::now();

        while (!stop()) {
            (void)zmq::poll(item, 1, stale_after);
            const auto now = std::chrono::steady_clock::now();

            if (!(item[0].revents & ZMQ_POLLIN)) {
                if (now - last_packet > stale_after) {
                    raise_error_("[ZmqCamera] No data from " + std::string(ENDPOINT)
                                 + " for " + std::to_string(RECV_TIMEOUT_MS) + "ms");
                    break;
                }
                continue;
            }

            zmq::message_t msg;
            if (!sock_.recv(msg, zmq::recv_flags::none)) continue;
            last_packet = now;

            // Server appends two little-endian float32 fields (pitch, roll —
            // degrees) after the RGB payload. See g1_files/realsense_raw_rgb.py.
            constexpr std::size_t EXPECTED = FRAME_SIZE + 2 * sizeof(float);
            if (msg.size() != EXPECTED) {
                raise_error_("[ZmqCamera] Unexpected frame size: "
                             + std::to_string(msg.size()) + " (expected "
                             + std::to_string(EXPECTED) + ")");
                break;
            }

            const uint8_t* rgb = msg.data<uint8_t>();
            float pitch_deg = 0.0f, roll_deg = 0.0f;
            std::memcpy(&pitch_deg, rgb + FRAME_SIZE,                 sizeof(float));
            std::memcpy(&roll_deg,  rgb + FRAME_SIZE + sizeof(float), sizeof(float));

            // Preview sees every frame (including during pause), same as CameraRecorder.
            if (preview_) {
                preview_->push_rgb(rgb, FRAME_WIDTH, FRAME_HEIGHT, FRAME_STRIDE);
                preview_->push_imu(pitch_deg, roll_deg);
            }

            if (pause()) continue;

            // camera_timestamp_ms = -1: the server doesn't forward the RealSense device timestamp
            std::ostringstream row;
            row << collection_id() << "," << frame_count << ",-1," << Time::ts()
                << "," << std::fixed << std::setprecision(2) << pitch_deg
                << "," << std::fixed << std::setprecision(2) << roll_deg;
            csv_.write_line(row.str());

            std::ostringstream fn;
            fn << frames_dir << "/frame_"
               << std::setfill('0') << std::setw(6) << frame_count << ".raw";

            if (!write_bytes(fn.str(), rgb, FRAME_SIZE)) {
                raise_error_("[ZmqCamera] Failed to write " + fn.str());
                break;
            }

            ++frame_count;
        }

        csv_.close();
        try { sock_.close(); } catch (...) {}
    } catch (const std::exception& e) {
        raise_error_(std::string("[ZmqCamera] ") + e.what());
    }
}
