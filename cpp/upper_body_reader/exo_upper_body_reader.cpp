#include "exo_upper_body_reader.hpp"

#include "arm_reader/dynamixel/dynamixel_arm.hpp"

#include "../utils/repo_constants.hpp"
#include "constants.hpp"

#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
std::string arm_csv_path(const std::string& recording_label,
                         const std::string& filename, bool enabled) {
  if (!enabled || recording_label.empty()) return "";
  return repo_constants::DATA_DIR + "/" + recording_label + "/" + filename;
}

std::unique_ptr<DynamixelArm> make_dynamixel_arm(
    bool enabled, const std::string& device, int baudrate, const char* side,
    const std::function<void(const std::string&)>& raise_error) {
  if (!enabled) return nullptr;
  if (device.empty()) {
    const std::string msg = std::string("[ExoUpperBodyReader] ") + side +
                            " arm is enabled but device path is empty";
    if (raise_error) raise_error(msg);
    throw std::invalid_argument(msg);
  }
  return std::make_unique<DynamixelArm>(device, baudrate, raise_error);
}
}  // namespace

ExoUpperBodyReader::ExoUpperBodyReader(
    const std::string& left_device, const std::string& right_device,
    int baudrate, const std::string& recording_label,
    bool left_enabled, bool right_enabled,
    const std::function<void(const std::string&)>& raise_error)
    : left(make_dynamixel_arm(left_enabled, left_device, baudrate, "left",
                              raise_error),
           arm_csv_path(recording_label, "left_arm.csv", left_enabled),
           raise_error, &converter_, true,
           arm_csv_path(recording_label, "left_command.csv", left_enabled)),
      right(make_dynamixel_arm(right_enabled, right_device, baudrate, "right",
                               raise_error),
            arm_csv_path(recording_label, "right_arm.csv", right_enabled),
            raise_error, &converter_, false,
            arm_csv_path(recording_label, "right_command.csv", right_enabled)) {}

void ExoUpperBodyReader::PrintRaw() const {
  const auto left_data = left.snapshot();
  const auto right_data = right.snapshot();

  auto p = [](uint16_t val) -> std::string {
    char buf[12];
    std::snprintf(buf, sizeof(buf), "  %5u |", val);
    return buf;
  };

  std::string out;
  out += "\033[H\033[J";
  out +=
      "__________________________________________________________________________\n";
  out +=
      " L_sh_pit | L_sh_rol | L_sh_yaw |  L_elbow | L_wr_rol | L_wr_pit | "
      "L_wr_yaw\n";
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) out += p(left_data.data[i]);
  out += "\n";
  out +=
      " R_sh_pit | R_sh_rol | R_sh_yaw |  R_elbow | R_wr_rol | R_wr_pit | "
      "R_wr_yaw\n";
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) out += p(right_data.data[i]);
  out += "\n";
  std::cout << out << std::flush;
}

void ExoUpperBodyReader::collect_loop(
    const std::function<int()>& collection_id,
    const std::function<bool()>& stop,
    const std::function<bool()>& pause) {
  std::thread left_thread([&] { left.collect_loop(collection_id, stop, pause); });
  std::thread right_thread(
      [&] { right.collect_loop(collection_id, stop, pause); });

  left_thread.join();
  right_thread.join();
}
