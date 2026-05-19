#include "usb_camera_recorder.hpp"

using namespace camera_frame;
using namespace usb_camera_constants;

#include "camera/preview_server.hpp"
#include "utils/repo_constants.hpp"
#include "utils/time.hpp"

#include <librealsense2/rs.hpp>

#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <fstream>

namespace {
    void push_to_preview(PreviewServer* preview, const rs2::video_frame& frame) {
        if (!preview) return;
        preview->push_rgb(static_cast<const uint8_t*>(frame.get_data()),
                          frame.get_width(),
                          frame.get_height(),
                          frame.get_stride_in_bytes());
    }
}

UsbCameraRecorder::UsbCameraRecorder(const std::string& recording_label,
                                     const std::function<void(const std::string&)>& raise_error,
                                     PreviewServer* preview)
    : output_dir_(repo_constants::DATA_DIR + "/" + recording_label),
      raise_error_(raise_error),
      csv_(output_dir_ + "/camera.csv", "collection_id,frame_number,camera_timestamp_ms,host_timestamp,pitch_deg,roll_deg"),
      pipe_(context_),
      preview_(preview) {
    try {
        std::filesystem::create_directories(output_dir_ + "/frames");

        if (context_.query_devices().size() == 0) {
            raise_error_("[Camera] No camera connected");
            return;
        }

        context_.set_devices_changed_callback([this](rs2::event_information info) {
            if (info.was_removed(device_)) {
                raise_error_("[Camera] Camera disconnected");
            }
        });

        // Try color + accel first (D435i). If the device has no IMU, or the
        // accel rate isn't resolvable on this link, fall back to color only —
        // pitch/roll will just stay at 0.
        auto start_with_accel = [&]() -> bool {
            try {
                rs2::config cfg;
                cfg.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_RGB8, FRAMERATE);
                cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, ACCEL_FRAMERATE);
                device_ = pipe_.start(cfg).get_device();
                return true;
            } catch (const std::exception& e) {
                std::cerr << "[Camera] accel start failed: " << e.what() << "\n";
                return false;
            }
        };
        if (!start_with_accel()) {
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_RGB8, FRAMERATE);
            device_ = pipe_.start(cfg).get_device();
            std::cerr << "[Camera] IMU unavailable — pitch/roll will read 0\n";
        }
    } catch (const std::exception& e) {
        raise_error_(std::string("[Camera] ") + e.what());
    }
}

bool UsbCameraRecorder::write_frame(const std::string& filename, const uint8_t* data) const {
    std::ofstream out(filename, std::ios::binary);
    if (!out) return false;

    out.write(reinterpret_cast<const char*>(data),
              static_cast<std::streamsize>(FRAME_SIZE));
    return static_cast<bool>(out);
}

void UsbCameraRecorder::collect_loop(const std::function<int()>&  collection_id,
                                     const std::function<bool()>& stop,
                                     const std::function<bool()>& pause) {
    try {
        const std::string frames_dir = output_dir_ + "/frames";
        auto camera_disconnected = [this] {
            return !device_ || !device_.is_connected();
        };

        for (int i = 0; i < WARMUP_COUNT && !stop(); ++i) {
            rs2::frameset frames;
            if (!pipe_.try_wait_for_frames(&frames, FRAME_TIMEOUT_MS) && camera_disconnected()) {
                raise_error_("[Camera] Camera disconnected");
                break;
            }
        }

        int frame_count = 0;
        // EMA-smoothed; pitch: level → ~0°, sky → negative, floor → positive.
        // roll: level → ~0°, tilted right → positive, left → negative.
        float pitch_deg = 0.0f;
        float roll_deg  = 0.0f;
        constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

        while (!stop()) {
            rs2::frameset frames;
            if (!pipe_.try_wait_for_frames(&frames, FRAME_TIMEOUT_MS)) {
                if (camera_disconnected()) {
                    raise_error_("[Camera] Camera disconnected");
                    break;
                }
                continue;
            }
            rs2::video_frame color = frames.get_color_frame();
            if (!color) continue;

            if (auto accel = frames.first_or_default(RS2_STREAM_ACCEL)) {
                auto a = accel.as<rs2::motion_frame>().get_motion_data();
                // D435i camera frame: +X right, +Y down, +Z forward.
                float ip = std::atan2(a.z, std::sqrt(a.x * a.x + a.y * a.y)) * RAD2DEG;
                // Camera is mounted rotated 180° around its forward axis, so
                // "level" gravity reads with ay negative — flip both components.
                float ir = std::atan2(-a.x, -a.y) * RAD2DEG;
                pitch_deg = 0.9f * pitch_deg + 0.1f * ip;
                roll_deg  = 0.9f * roll_deg  + 0.1f * ir;
                if (preview_) preview_->push_imu(pitch_deg, roll_deg);
            }

            push_to_preview(preview_, color);
            if (pause()) continue;

            std::ostringstream row;
            row << collection_id() << "," << frame_count << ","
                << std::fixed << std::setprecision(3) << color.get_timestamp() << ","
                << Time::ts() << ","
                << std::fixed << std::setprecision(2) << pitch_deg << ","
                << std::fixed << std::setprecision(2) << roll_deg;
            csv_.write_line(row.str());

            const uint8_t* data = static_cast<const uint8_t*>(color.get_data());

            std::ostringstream filename;
            filename << frames_dir << "/frame_"
                     << std::setfill('0') << std::setw(6) << frame_count
                     << ".raw";

            if (!write_frame(filename.str(), data)) {
                raise_error_("[Camera] Failed to write " + filename.str());
                break;
            }

            ++frame_count;
        }

        csv_.close();
        pipe_.stop();
    } catch (const std::exception& e) {
        raise_error_(std::string("[Camera] ") + e.what());
    }
}
