#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "../gamepad.hpp"
#include "../utils/DataBuffer.hpp"
#include "model/g1Enums.hpp"
#include "model/g1Structs.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

class G1Robot {
 protected:
  Mode mode_pr_;
  uint8_t mode_machine_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;
  DataBuffer<OdomState> odom_state_buffer_;

  // Hook fired at the end of LowStateHandler
  virtual void on_state_update() {}

 private:
  int counter_;
  Gamepad gamepad_;
  REMOTE_DATA_RX rx_;

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;
  ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> odom_subscriber_;
  ThreadPtr command_writer_ptr_;

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

 public:
  G1Robot(std::string networkInterface, bool isSimulation);
  virtual ~G1Robot() = default;

  void imuTorsoHandler(const void *message);
  void LowStateHandler(const void *message);
  void OdomStateHandler(const void *message);
  void LowCommandWriter();
  std::optional<MotorState> getMotorStateSnapshot() const;
};
