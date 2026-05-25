#pragma once

#include <functional>
#include <string>

#include "g1/g1Controller.hpp"
#include "arm_reader/arm_reader.hpp"
#include "upper_body_reader.hpp"

struct G1UpperBodyReaderConfig {
  G1ControllerConfig controller;
  std::string left_device;
  std::string right_device;
  int baudrate = 0;
};

class G1UpperBodyReader : public UpperBodyReader {
 public:
  G1UpperBodyReader(
      const G1UpperBodyReaderConfig& config,
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
