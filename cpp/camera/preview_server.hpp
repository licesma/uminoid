#pragma once

// Tiny in-process HTTP server that publishes a heavily downscaled, low-quality
// MJPEG stream of the latest RGB frame seen by the recorder. Intended for
// view-only monitoring from other machines on the same Tailnet while the CLI
// recorder is running. The CLI keeps sole ownership of keyboard controls —
// this endpoint never accepts input.
//
// Recommended binding: the recording machine's tailscale0 address (e.g.
// 100.x.y.z). Binding to 0.0.0.0 also works but additionally exposes the
// stream on any physical LAN the host sits on.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class PreviewServer {
public:
    struct Config {
        std::string bind_host = "0.0.0.0";
        int port          = 8080;
        int out_width     = 320;   // downscaled preview resolution
        int out_height    = 240;
        int jpeg_quality  = 40;    // 1..95, lower = smaller/uglier
        int max_fps       = 10;    // cap preview frame rate
    };

    PreviewServer(Config cfg,
                  std::function<void(const std::string&)> on_error);
    ~PreviewServer();

    PreviewServer(const PreviewServer&)            = delete;
    PreviewServer& operator=(const PreviewServer&) = delete;

    void start();
    void stop();

    // Non-blocking. Copies the RGB frame so the recorder hot loop is never
    // slowed by the encoder or by slow HTTP clients. Latest-wins: if the
    // encoder hasn't consumed the previous frame, it is overwritten.
    // `stride` is bytes per row of `rgb` (usually width * 3).
    void push_rgb(const uint8_t* rgb, int width, int height, int stride);

    // Latest camera tilt (degrees), served at GET /imu as JSON.
    void push_imu(float pitch_deg, float roll_deg);

private:
    void accept_loop_();
    void encode_loop_();
    void serve_client_(int fd);

    Config cfg_;
    std::function<void(const std::string&)> on_error_;

    std::atomic<bool> running_{false};
    int listen_fd_ = -1;

    std::thread accept_thread_;
    std::thread encode_thread_;

    // Staged raw RGB (producer: recorder; consumer: encoder)
    std::mutex              stage_mu_;
    std::condition_variable stage_cv_;
    std::vector<uint8_t>    staged_rgb_;
    int                     staged_w_      = 0;
    int                     staged_h_      = 0;
    int                     staged_stride_ = 0;
    uint64_t                staged_seq_    = 0;
    uint64_t                consumed_seq_  = 0;

    // Latest encoded JPEG (producer: encoder; consumer: client threads)
    std::mutex              jpeg_mu_;
    std::condition_variable jpeg_cv_;
    std::vector<uint8_t>    latest_jpeg_;
    uint64_t                latest_jpeg_seq_ = 0;

    std::atomic<float>      latest_pitch_deg_{0.f};
    std::atomic<float>      latest_roll_deg_{0.f};
};
