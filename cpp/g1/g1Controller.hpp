#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "../utils/csv_saver.hpp"
#include "../utils/metadata_loader.hpp"
#include "upper_body_reader/arm_reader/skeleton_arm.hpp"
#include "upper_body_reader/constants.hpp"
#include "../utils/bounds_loader.hpp"
#include "amo/amo_bridge.hpp"
#include "g1Robot.hpp"

#ifndef READER_BOUNDS_PATH
#define READER_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
#endif

#ifndef DYNAMIXEL_BOUNDS_PATH
#define DYNAMIXEL_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
#endif

struct G1JointReading {
  G1JointIndex joint;
  double netAngle;
  bool is_valid;
};

using ArmReadings = std::array<G1JointReading, ARM_JOINT_COUNT>;

class G1Controller : public G1Robot {
 private:
  double control_dt_;
  double max_target_velocity_;
  std::array<double, G1_NUM_MOTOR> commanded_targets_;
  CsvSaver left_measured_csv_;
  CsvSaver right_measured_csv_;
  CsvSaver left_command_csv_;
  CsvSaver right_command_csv_;
  std::mutex update_mutex_;
  JointsReadingMetadata metadata_;
  JointBounds bounds_;
  JointBounds reader_bounds_;
  bool left_enabled_;
  bool right_enabled_;

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

  ArmReadings decode_arm(const ArmLine& sample, bool from_left) const;
  double toG1Angle(G1JointReading reading);
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
  G1Controller(std::string networkInterface, bool isSimulation,
               const JointsReadingMetadata& metadata,
               const JointBounds& reader_bounds,
               const std::string& recording_label,
               bool left_enabled, bool right_enabled,
               const std::function<void(const std::string&)>& raise_error);
  ~G1Controller() override = default;

  bool initialize_targets_from_robot_state(
      const std::function<bool()>& stop_requested);
  void process_arm_sample(const ArmLine& sample, bool from_left,
                          int collection_id, bool record);

  void handle_key(char key);
};
