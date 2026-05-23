#include "g1Controller.hpp"

#include "g1/model/g1Enums.hpp"
#include "g1/model/g1Values.hpp"
#include "utils/circular_math.hpp"
#include "utils/repo_constants.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace {

// Linear blend window applied to AMO actions right after handoff. At t=0 the
// commanded targets equal initial_pose (the ramp endpoint), at t=duration they
// equal pure policy output. Short enough not to soft-mute real balance
// corrections, long enough to absorb the first non-zero policy residual.
constexpr double AMO_BLEND_DURATION_S = 1.5;

// Same idea, applied to operator arm tracking after the first space press.
// At t=0 the per-joint tracking target equals initial_pose; at t=duration it
// equals the operator's pose; in between, a linear blend. The existing
// per-sample velocity clamp still runs on top, so a large operator-vs-parked
// gap that exceeds the ramp's implied velocity is bounded by the clamp.
constexpr double ARM_FOLLOW_RAMP_DURATION_S = 3.0;

std::string csv_header() {
  std::string h = "collection_id,timestamp,host_timestamp";
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i)
    h += ",joint_" + std::to_string(i);
  return h;
}

CsvSaver make_csv_saver(const std::string& recording_label,
                        const std::string& filename) {
  if (recording_label.empty()) return CsvSaver{};
  const std::string dir = repo_constants::DATA_DIR + "/" + recording_label;
  std::filesystem::create_directories(dir);
  return CsvSaver(dir + "/" + filename, csv_header());
}

MotorCommand make_motor_command(
    const std::array<double, G1_NUM_MOTOR>& commanded_targets) {
  MotorCommand motor_command;
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    motor_command.tau_ff.at(i) = 0.0;
    motor_command.q_target.at(i) = commanded_targets.at(i);
    motor_command.dq_target.at(i) = 0.0;
    motor_command.kp.at(i) = stiffness[i];
    motor_command.kd.at(i) = damping[i];
  }
  return motor_command;
}

}  // namespace

G1Controller::G1Controller(std::string networkInterface, bool isSimulation,
                           const std::string& recording_label,
                           bool left_enabled, bool right_enabled,
                           const std::function<void(const std::string&)>& raise_error)
    : G1Robot(networkInterface, isSimulation),
      control_dt_(0.002),
      max_target_velocity_(2.0),
      commanded_targets_{},
      left_measured_csv_(left_enabled ? make_csv_saver(recording_label, "left_measured.csv") : CsvSaver{}),
      right_measured_csv_(right_enabled ? make_csv_saver(recording_label, "right_measured.csv") : CsvSaver{}),
      left_command_csv_(left_enabled ? make_csv_saver(recording_label, "left_command.csv") : CsvSaver{}),
      right_command_csv_(right_enabled ? make_csv_saver(recording_label, "right_command.csv") : CsvSaver{}),
      left_arm_csv_(left_enabled ? make_csv_saver(recording_label, "left_arm.csv") : CsvSaver{}),
      right_arm_csv_(right_enabled ? make_csv_saver(recording_label, "right_arm.csv") : CsvSaver{}),
      left_enabled_(left_enabled),
      right_enabled_(right_enabled),
      amo_bridge_([this](const AmoAction& action) { apply_amo_action(action); },
                  raise_error) {
}

void G1Controller::record_arm(const ArmLine& sample, const MotorState& state,
                              const MotorCommand& command, bool from_left,
                              int collection_id) {
  CsvSaver& measured = from_left ? left_measured_csv_ : right_measured_csv_;
  CsvSaver& cmd = from_left ? left_command_csv_ : right_command_csv_;
  CsvSaver& arm = from_left ? left_arm_csv_ : right_arm_csv_;
  if (!measured || !cmd || !arm) return;

  const auto& joints = from_left ? LEFT_ARM_JOINTS : RIGHT_ARM_JOINTS;
  const std::string prefix = std::to_string(collection_id) + "," +
                             std::to_string(sample.timestamp) + "," +
                             std::to_string(sample.host_timestamp);
  std::ostringstream measured_row;
  std::ostringstream command_row;
  std::ostringstream arm_row;
  measured_row << prefix;
  command_row << prefix;
  arm_row << prefix;

  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    const int idx = static_cast<int>(joints[i]);
    measured_row << "," << state.q.at(idx);
    command_row << "," << command.q_target.at(idx);
    arm_row << "," << sample.data[i];
  }

  measured.write_line(measured_row.str());
  cmd.write_line(command_row.str());
  arm.write_line(arm_row.str());
}

bool G1Controller::initialize_targets_from_robot_state(
    const std::function<bool()>& stop_requested) {
  std::array<double, G1_NUM_MOTOR> initial_q{};
  while (!stop_requested()) {
    const std::shared_ptr<const MotorState> first_motor_state =
        motor_state_buffer_.GetData();
    if (!first_motor_state) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      initial_q.at(i) = first_motor_state->q.at(i);
    }
    break;
  }
  if (stop_requested()) return false;

  // Ramp targets come from g1Values.hpp::initial_pose (legs in AMO's default
  // crouch so the first policy tick is in-distribution; both elbows parked at
  // DISABLED_ARM_ELBOW_Q so neither arm dangles at q=0 before arm following
  // takes over).
  const std::array<double, G1_NUM_MOTOR>& final_q = initial_pose;

  const double duration = 3.0;
  const int num_steps = static_cast<int>(duration / control_dt_);
  mode_pr_ = Mode::PR;
  for (int step = 0; step <= num_steps; ++step) {
    if (stop_requested()) return false;
    const double ratio = std::clamp(
        static_cast<double>(step) / static_cast<double>(num_steps), 0.0, 1.0);
    {
      std::lock_guard<std::mutex> lock(update_mutex_);
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        commanded_targets_.at(i) =
            (1.0 - ratio) * initial_q.at(i) + ratio * final_q.at(i);
      }
      MotorCommand motor_command_tmp = make_motor_command(commanded_targets_);
      motor_command_buffer_.SetData(motor_command_tmp);
    }
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<int>(control_dt_ * 1e6)));
  }

  amo_handoff_time_ = std::chrono::steady_clock::now();
  amo_ready_.store(true);
  return true;
}

void G1Controller::process_arm_sample(const ArmLine& sample, bool from_left,
                                      int collection_id, bool record) {
  if (from_left ? !left_enabled_ : !right_enabled_) return;
  if (record && !arm_following_started_.load(std::memory_order_relaxed)) {
    arm_handoff_time_ = std::chrono::steady_clock::now();
    arm_following_started_.store(true, std::memory_order_relaxed);
  }
  if (!arm_following_started_.load(std::memory_order_relaxed)) return;
  const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
  if (!ms) return;

  const std::array<double, ARM_JOINT_COUNT> angles =
      converter_.convert(sample, from_left);
  const auto& joints = from_left ? LEFT_ARM_JOINTS : RIGHT_ARM_JOINTS;
  const double max_step = max_target_velocity_ * control_dt_;
  const double t_since_handoff = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - arm_handoff_time_).count();
  const double ramp_alpha =
      std::clamp(t_since_handoff / ARM_FOLLOW_RAMP_DURATION_S, 0.0, 1.0);

  std::lock_guard<std::mutex> lock(update_mutex_);
  MotorCommand motor_command_tmp = make_motor_command(commanded_targets_);

  mode_pr_ = Mode::PR;
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    if (std::isnan(angles[i])) continue;
    const int joint_index = static_cast<int>(joints[i]);
    const double desired_target = angles[i];
    const double tracking_target =
        ramp_alpha < 1.0
            ? (1.0 - ramp_alpha) * initial_pose[joint_index] +
                  ramp_alpha * desired_target
            : desired_target;
    commanded_targets_.at(joint_index) += std::clamp(
        tracking_target - commanded_targets_.at(joint_index), -max_step,
        max_step);
    motor_command_tmp.q_target.at(joint_index) =
        commanded_targets_.at(joint_index);
  }

  if (record) record_arm(sample, *ms, motor_command_tmp, from_left, collection_id);
  motor_command_buffer_.SetData(motor_command_tmp);
}

void G1Controller::publish_amo_state() {
  const std::shared_ptr<const MotorState> motor = motor_state_buffer_.GetData();
  const std::shared_ptr<const ImuState>   imu   = imu_state_buffer_.GetData();
  if (!motor || !imu) return;

  AmoCommand cmd_snapshot;
  {
    std::lock_guard<std::mutex> lock(amo_command_mutex_);
    cmd_snapshot = amo_command_;
  }
  amo_bridge_.publish_state(amo_state_seq_++, *motor, *imu, imu->quat,
                            cmd_snapshot);
}

void G1Controller::handle_key(char key) {
  std::lock_guard<std::mutex> lock(amo_command_mutex_);
  switch (key) {
    case 'u': amo_command_.height += 0.05; break;
    case 'j': amo_command_.height -= 0.05; break;
    // case 'w': amo_command_.vx          += 0.05; break;
    // case 's': amo_command_.vx          -= 0.05; break;
    // case 'a': amo_command_.yaw_target  += 0.10; break;
    // case 'd': amo_command_.yaw_target  -= 0.10; break;
    // case 'q': amo_command_.vy          += 0.05; break;
    // case 'e': amo_command_.vy          -= 0.05; break;
    // case 'j': amo_command_.torso_yaw   += 0.10; break;
    // case 'u': amo_command_.torso_yaw   -= 0.10; break;
    // case 'k': amo_command_.torso_pitch += 0.05; break;
    // case 'i': amo_command_.torso_pitch -= 0.05; break;
    // case 'l': amo_command_.torso_roll  += 0.05; break;
    // case 'o': amo_command_.torso_roll  -= 0.05; break;
    // case '0': amo_command_ = AmoCommand{}; break;  // reset all
    default: break;  // unrecognized key -- ignore silently
  }
}

void G1Controller::apply_amo_action(const AmoAction& action) {
  if (!amo_ready_.load()) return;

  const double t_since_handoff = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - amo_handoff_time_).count();
  const double alpha =
      std::clamp(t_since_handoff / AMO_BLEND_DURATION_S, 0.0, 1.0);

  std::lock_guard<std::mutex> lock(update_mutex_);
  for (size_t i = 0; i < AMO_ACTION.size(); ++i) {
    const int idx = static_cast<int>(AMO_ACTION[i]);
    commanded_targets_[idx] =
        (1.0 - alpha) * initial_pose[idx] + alpha * action.q_target[i];
  }
  motor_command_buffer_.SetData(make_motor_command(commanded_targets_));
}

void G1Controller::on_state_update() {
  // Do NOT add anything that can block, this runs DDS callback 500Hz
  if (!amo_ready_.load()) return;
  publish_amo_state();
}
