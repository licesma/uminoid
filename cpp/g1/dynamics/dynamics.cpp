#include "g1/dynamics/dynamics.hpp"

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cctype>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "g1/model/g1Enums.hpp"
#include "g1/model/g1Values.hpp"

#ifndef G1_URDF_PATH
#error "G1_URDF_PATH must be defined (path to g1_body29_hand14.urdf)"
#endif

namespace {

// URDF joint name -> G1JointIndex for the 14 arm joints that remain after the
// legs, waist, and hands are locked out of the reduced model. Order is
// irrelevant; the mapping is by name.
const std::array<std::pair<const char*, G1JointIndex>, 14> kArmJoints = {{
    {"left_shoulder_pitch_joint", LeftShoulderPitch},
    {"left_shoulder_roll_joint", LeftShoulderRoll},
    {"left_shoulder_yaw_joint", LeftShoulderYaw},
    {"left_elbow_joint", LeftElbow},
    {"left_wrist_roll_joint", LeftWristRoll},
    {"left_wrist_pitch_joint", LeftWristPitch},
    {"left_wrist_yaw_joint", LeftWristYaw},
    {"right_shoulder_pitch_joint", RightShoulderPitch},
    {"right_shoulder_roll_joint", RightShoulderRoll},
    {"right_shoulder_yaw_joint", RightShoulderYaw},
    {"right_elbow_joint", RightElbow},
    {"right_wrist_roll_joint", RightWristRoll},
    {"right_wrist_pitch_joint", RightWristPitch},
    {"right_wrist_yaw_joint", RightWristYaw},
}};

}  // namespace

// Per-arm-joint location of the desired angle in the reduced model's q vector
// and of its torque in the rnea output, paired with the destination
// G1JointIndex in the full 29-joint command.
struct Dynamics::PinochioImplementation {
  pinocchio::Model model;
  pinocchio::Data data;
  struct ArmSlot {
    int g1_idx;  // index into the 29-joint command arrays
    int q_idx;   // offset in reduced model q
    int v_idx;   // offset in reduced model v / rnea tau
  };
  std::vector<ArmSlot> arms;

  explicit PinochioImplementation(pinocchio::Model reduced)
      : model(std::move(reduced)), data(model) {}
};

DynamicsModel parse_dynamics_model(const std::string& s) {
  std::string lower = s;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (lower == "baseline") return DynamicsModel::Baseline;
  if (lower == "psi0") return DynamicsModel::Psi0;
  throw std::invalid_argument("unknown dynamics model '" + s +
                              "' (expected 'baseline' or 'psi0')");
}

Dynamics::Dynamics(DynamicsModel model) : model_(model) {
  if (model_ == DynamicsModel::Baseline) {
    std::copy(stiffness.begin(), stiffness.end(), kp_.begin());
    std::copy(damping.begin(), damping.end(), kd_.begin());
    return;
  }

  std::copy(psi0_stiffness.begin(), psi0_stiffness.end(), kp_.begin());
  std::copy(psi0_damping.begin(), psi0_damping.end(), kd_.begin());

  // Load the full fixed-base model, then lock every actuated joint that is not
  // one of the 14 arm joints (legs, waist, hands). This mirrors the reduced
  // model Psi-0 builds in compute_tau.py, but identifying the kept joints by
  // name rather than enumerating the locked ones.
  pinocchio::Model full;
  pinocchio::urdf::buildModel(G1_URDF_PATH, full);

  std::unordered_set<std::string> arm_names;
  for (const auto& [name, g1_idx] : kArmJoints) arm_names.insert(name);

  std::vector<pinocchio::JointIndex> locked;
  for (pinocchio::JointIndex jid = 1;
       jid < static_cast<pinocchio::JointIndex>(full.njoints); ++jid) {
    if (arm_names.find(full.names[jid]) == arm_names.end()) {
      locked.push_back(jid);
    }
  }

  const Eigen::VectorXd reference = Eigen::VectorXd::Zero(full.nq);
  pinocchio::Model reduced;
  pinocchio::buildReducedModel(full, locked, reference, reduced);

  pin_ = std::make_unique<PinochioImplementation>(std::move(reduced));

  for (const auto& [name, g1_idx] : kArmJoints) {
    if (!pin_->model.existJointName(name)) {
      throw std::runtime_error(
          std::string("arm joint '") + name +
          "' missing from reduced Pinocchio model");
    }
    const pinocchio::JointIndex jid = pin_->model.getJointId(name);
    pin_->arms.push_back({static_cast<int>(g1_idx),
                          pin_->model.idx_qs[jid],
                          pin_->model.idx_vs[jid]});
  }
}

Dynamics::~Dynamics() = default;
Dynamics::Dynamics(Dynamics&&) noexcept = default;
Dynamics& Dynamics::operator=(Dynamics&&) noexcept = default;

std::array<float, G1_NUM_MOTOR> Dynamics::getFFTau(
    const std::array<double, G1_NUM_MOTOR>& q_target) const {
  std::array<float, G1_NUM_MOTOR> tau_ff{};
  if (model_ == DynamicsModel::Baseline) return tau_ff;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(pin_->model.nq);
  for (const auto& arm : pin_->arms) {
    q[arm.q_idx] = q_target[arm.g1_idx];
  }
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(pin_->model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Zero(pin_->model.nv);

  const Eigen::VectorXd tau = pinocchio::rnea(pin_->model, pin_->data, q, v, a);
  for (const auto& arm : pin_->arms) {
    tau_ff[arm.g1_idx] = static_cast<float>(tau[arm.v_idx]);
  }
  return tau_ff;
}
