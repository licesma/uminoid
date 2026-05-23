#pragma once

#include <functional>
#include <string>

#include "g1/g1Controller.hpp"
#include "arm_reader/arm_reader.hpp"
#include "upper_body_reader.hpp"

class G1UpperBodyReader : public UpperBodyReader {
 public:
  G1UpperBodyReader(
      std::string networkInterface, const std::string& left_device,
      const std::string& right_device, int baudrate, bool isSimulation,
      const std::string& recording_label,
      bool left_enabled, bool right_enabled,
      const std::function<void(const std::string&)>& raise_error);
  ~G1UpperBodyReader() override = default;

  /** Drives teleoperation until `stop()` returns true. While `pause()` returns
   *  true, the robot keeps tracking the operator's arms but no CSV rows are
   *  written. */
  void collect_loop(const std::function<int()>&  collection_id,
                    const std::function<bool()>& stop,
                    const std::function<bool()>& pause = [] { return false; })
      override;

  void handle_key(char key) override;

 private:
  void arm_listener_loop(ArmReader& reader, bool from_left,
                         const std::function<int()>&  collection_id,
                         const std::function<bool()>& stop,
                         const std::function<bool()>& pause);

  G1Controller controller_;
  ArmReader left_;
  ArmReader right_;
};
