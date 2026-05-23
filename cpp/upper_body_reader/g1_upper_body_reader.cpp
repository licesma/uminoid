#include "g1_upper_body_reader.hpp"

#include "arm_reader/dynamixel/dynamixel_arm.hpp"

#include <stdexcept>
#include <thread>

namespace {
std::unique_ptr<DynamixelArm> make_dynamixel_arm(
    bool enabled, const std::string& device, int baudrate, const char* side,
    const std::function<void(const std::string&)>& raise_error) {
  if (!enabled) return nullptr;
  if (device.empty()) {
    const std::string msg = std::string("[G1UpperBodyReader] ") + side +
                            " arm is enabled but device path is empty";
    if (raise_error) raise_error(msg);
    throw std::invalid_argument(msg);
  }
  return std::make_unique<DynamixelArm>(device, baudrate, raise_error);
}
}  // namespace

G1UpperBodyReader::G1UpperBodyReader(
    std::string networkInterface, const std::string& left_device,
    const std::string& right_device, int baudrate, bool isSimulation,
    const std::string& recording_label,
    bool left_enabled, bool right_enabled,
    const std::function<void(const std::string&)>& raise_error)
    : controller_(networkInterface, isSimulation, recording_label,
                  left_enabled, right_enabled, raise_error),
      left_(make_dynamixel_arm(left_enabled, left_device, baudrate, "left",
                               raise_error),
            "", raise_error),
      right_(make_dynamixel_arm(right_enabled, right_device, baudrate, "right",
                                raise_error),
             "", raise_error) {}

void G1UpperBodyReader::collect_loop(
    const std::function<int()>&  collection_id,
    const std::function<bool()>& stop,
    const std::function<bool()>& pause) {
  if (!controller_.initialize_targets_from_robot_state(stop)) return;

  std::thread left_thread(&G1UpperBodyReader::arm_listener_loop, this,
                          std::ref(left_), true, std::cref(collection_id),
                          std::cref(stop), std::cref(pause));
  std::thread right_thread(&G1UpperBodyReader::arm_listener_loop, this,
                           std::ref(right_), false, std::cref(collection_id),
                           std::cref(stop), std::cref(pause));

  left_thread.join();
  right_thread.join();
}

void G1UpperBodyReader::handle_key(char key) {
  controller_.handle_key(key);
}

void G1UpperBodyReader::arm_listener_loop(
    ArmReader& reader, bool from_left,
    const std::function<int()>&  collection_id,
    const std::function<bool()>& stop,
    const std::function<bool()>& pause) {
  while (!stop()) {
    auto fresh_line = reader.wait_for_next();
    if (!fresh_line) break;

    controller_.process_arm_sample(*fresh_line, from_left, collection_id(),
                                   !pause());
  }
}
