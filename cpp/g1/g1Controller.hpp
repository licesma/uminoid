#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "../utils/csv_saver.hpp"
#include "upper_body_reader/arm_reader/arm_angle_converter.hpp"
#include "upper_body_reader/arm_reader/skeleton_arm.hpp"
#include "upper_body_reader/constants.hpp"
#include "amo/amo_bridge.hpp"
#include "dynamics/dynamics.hpp"
#include "g1Robot.hpp"

#ifndef DYNAMIXEL_BOUNDS_PATH
#define DYNAMIXEL_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
#endif

struct G1ControllerConfig {
  std::string network_interface;
  bool is_simulation = false;
  std::string recording_label;  // empty disables CSV recording
  bool left_enabled = false;
  bool right_enabled = false;
  DynamicsModel dynamics_model = DynamicsModel::Baseline;
};

class G1Controller : public G1Robot {
 private:
  double control_dt_;
  double max_target_velocity_;
  std::array<double, G1_NUM_MOTOR> commanded_targets_;
  CsvSaver left_measured_csv_;
  CsvSaver right_measured_csv_;
  CsvSaver left_command_csv_;
  CsvSaver right_command_csv_;
  CsvSaver left_arm_csv_;
  CsvSaver right_arm_csv_;
  std::mutex update_mutex_;
  ArmAngleConverter converter_;
  bool left_enabled_;
  bool right_enabled_;
  Dynamics dynamics_;

  // Builds a MotorCommand from the given per-joint targets, filling kp/kd and
  // the feed-forward torque from the active dynamics model.
  MotorCommand make_motor_command(
      const std::array<double, G1_NUM_MOTOR>& commanded_targets) const;

  // AMO sidecar plumbing.
  AmoBridge  amo_bridge_;
  AmoCommand amo_command_;         
  std::mutex amo_command_mutex_;    
  uint64_t   amo_state_seq_ = 0;    
  // Stays false until initialize_targets_from_robot_state has finished.
  std::atomic<bool> amo_ready_{false};
  // Latches true the first time process_arm_sample is called while not paused
  // (i.e. when the user starts the first collection with space). Stays true
  // for the rest of the session, including across subsequent pauses, so the
  // robot only begins tracking the operator's arms once they've explicitly
  // started collecting.
  std::atomic<bool> arm_following_started_{false};
  // Stamped when arm_following_started_ flips true. process_arm_sample blends
  // initial_pose into the operator's pose over a short window so the arms
  // don't lurch on the very first space press.
  std::chrono::steady_clock::time_point arm_handoff_time_{};
  // Set immediately before amo_ready_ flips true. apply_amo_action blends
  // the C++ ramp endpoint (initial_pose) into the policy's commanded targets
  // over a short window so the very first AMO tick is not a position step.
  std::chrono::steady_clock::time_point amo_handoff_time_{};

  void record_arm(const ArmLine& sample, const MotorState& state,
                  const MotorCommand& command, bool from_left,
                  int collection_id);

  // Called from LowStateHandler at ~500 Hz via the on_state_update() override.
  void publish_amo_state();

  // Called from AmoBridge's receive thread on each successfully unpacked frame.
  void apply_amo_action(const AmoAction& action);

  // Override of G1Robot::on_state_update -- fires from LowStateHandler.
  void on_state_update() override;

 public:
  G1Controller(const G1ControllerConfig& config,
               const std::function<void(const std::string&)>& raise_error);
  ~G1Controller() override = default;

  bool initialize_targets_from_robot_state(
      const std::function<bool()>& stop_requested);
  void process_arm_sample(const ArmLine& sample, bool from_left,
                          int collection_id, bool record);

  void handle_key(char key);
};
