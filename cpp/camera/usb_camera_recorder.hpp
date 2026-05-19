#pragma once

#include <functional>
#include <librealsense2/rs.hpp>
#include <string>

#include "camera/camera_recorder.hpp"
#include "utils/csv_saver.hpp"

class PreviewServer;

namespace usb_camera_constants {
    constexpr int FRAMERATE = 30;
    constexpr int ACCEL_FRAMERATE = 100;  // D435i: 100/200/400 Hz (older fw) or 63/250 Hz (newer); 100 works on both
    constexpr unsigned int FRAME_TIMEOUT_MS = 5000;
    constexpr unsigned int WARMUP_COUNT = 30;
}

class UsbCameraRecorder : public CameraRecorder {
public:
    UsbCameraRecorder(const std::string& recording_label,
                      const std::function<void(const std::string&)>& raise_error,
                      PreviewServer* preview = nullptr);

    void collect_loop(const std::function<int()>&  collection_id,
                      const std::function<bool()>& stop,
                      const std::function<bool()>& pause) override;

private:
    bool write_frame(const std::string& filename, const uint8_t* data) const;

    std::string output_dir_;
    std::function<void(const std::string&)> raise_error_;
    CsvSaver csv_;
    rs2::context context_;
    rs2::pipeline pipe_;
    rs2::device device_;
    PreviewServer* preview_ = nullptr;
};
