#pragma once

#include "arm_reader/arm_reader.hpp"
#include "upper_body_reader.hpp"

#include <functional>
#include <string>

class ExoUpperBodyReader : public UpperBodyReader {
 public:
  /** Construct with DynamixelArms (USB/U2D2). A side that is disabled (or has
   *  an empty device path) is skipped entirely -- no reader thread, no CSV. */
  ExoUpperBodyReader(
      const std::string& left_device, const std::string& right_device,
      int baudrate, const std::string& recording_label = "",
      bool left_enabled = true, bool right_enabled = true,
      const std::function<void(const std::string&)>& raise_error = nullptr);

  ~ExoUpperBodyReader() override = default;

  void PrintRaw() const;

  void collect_loop(const std::function<int()>&  collection_id,
                    const std::function<bool()>& stop,
                    const std::function<bool()>& pause = [] { return false; })
      override;

 private:
  ArmAngleConverter converter_;

 public:
  ArmReader left;
  ArmReader right;
};
