#pragma once

// AmoBridge: ZMQ link to the Python AMO sidecar.
//
//   state  : C++ PUB (bind)    -> Python SUB (connect)   tcp://127.0.0.1:5555
//   action : C++ SUB (connect) <- Python PUB (bind)      tcp://127.0.0.1:5556
//
// Wire format must match py/collect/amo_sidecar/wire_protocol.py byte-for-byte.

#include "g1/model/g1Enums.hpp"
#include "g1/model/g1Structs.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include <zmq.hpp>

// AMO 23-DoF *observation* order.
inline constexpr std::array<G1JointIndex, 23> AMO_OBSERVATION = {
    LeftHipPitch,    LeftHipRoll,    LeftHipYaw,    LeftKnee,
    LeftAnklePitch,  LeftAnkleRoll,
    RightHipPitch,   RightHipRoll,   RightHipYaw,   RightKnee,
    RightAnklePitch, RightAnkleRoll,
    WaistYaw,        WaistRoll,      WaistPitch,
    LeftShoulderPitch,  LeftShoulderRoll,  LeftShoulderYaw,  LeftElbow,
    RightShoulderPitch, RightShoulderRoll, RightShoulderYaw, RightElbow,
};

// AMO 15-DoF *action* order -> G1JointIndex
inline constexpr std::array<G1JointIndex, 15> AMO_ACTION = {
    LeftHipPitch,    LeftHipRoll,    LeftHipYaw,    LeftKnee,
    LeftAnklePitch,  LeftAnkleRoll,
    RightHipPitch,   RightHipRoll,   RightHipYaw,   RightKnee,
    RightAnklePitch, RightAnkleRoll,
    WaistYaw,        WaistRoll,      WaistPitch,
};

inline constexpr int AMO_LOWER_DOF = static_cast<int>(AMO_ACTION.size());


struct AmoCommand {
  double vx = 0.0;
  double yaw_target = 0.0;
  double vy = 0.0;
  double height = 0.0;
  double torso_yaw = 0.0;
  double torso_pitch = 0.0;
  double torso_roll = 0.0;
};

struct AmoAction {
  uint64_t seq = 0;     // echoes the state.seq used to compute this action
  uint64_t ts_ns = 0;   // sidecar publish time, monotonic ns
  std::array<double, AMO_LOWER_DOF> q_target{};
};

class AmoBridge {
 public:
  struct Config {
    std::string state_endpoint  = "tcp://127.0.0.1:5555"; 
    std::string action_endpoint = "tcp://127.0.0.1:5556"; 
  };

  using ActionCallback = std::function<void(const AmoAction&)>;
  using ErrorCallback  = std::function<void(const std::string&)>;

  AmoBridge(ActionCallback on_action, ErrorCallback raise_error);
  AmoBridge(ActionCallback on_action, Config cfg, ErrorCallback raise_error);
  ~AmoBridge();
  AmoBridge(const AmoBridge&) = delete;
  AmoBridge& operator=(const AmoBridge&) = delete;


  void publish_state(uint64_t seq,
                     const MotorState& motor_state,
                     const ImuState& imu_state,
                     const std::array<float, 4>& quat_wxyz,
                     const AmoCommand& cmd);

  // TODO(amo-bridge): remove these debug counters
  uint64_t debug_successful_action()  const { return debug_successful_action_.load(); }
  uint64_t debug_bad_frame_counts()   const { return debug_bad_frame_counts_.load(); }

 private:
  void receive_loop_();

  Config         cfg_;
  ActionCallback on_action_;
  ErrorCallback  raise_error_;
  zmq::context_t ctx_;
  zmq::socket_t  state_pub_;
  zmq::socket_t  action_sub_;

  // TODO(amo-bridge): remove once sidecar integration is trusted.
  std::atomic<uint64_t> debug_successful_action_{0};
  std::atomic<uint64_t> debug_bad_frame_counts_{0};

  std::atomic<bool> running_{false};
  std::thread receive_thread_;
};
