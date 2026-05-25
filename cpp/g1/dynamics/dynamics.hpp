#pragma once

#include <array>
#include <memory>
#include <string>

#include "g1/model/g1Structs.hpp"  // G1_NUM_MOTOR

// Selects which low-level joint dynamics the controller assembles into each
// MotorCommand.
//
//   Baseline : the historical behavior -- fixed PD gains, zero feed-forward.
//   Psi0     : Psi-0's behavior 
enum class DynamicsModel { Baseline, Psi0 };


DynamicsModel parse_dynamics_model(const std::string& s);

class Dynamics {
 public:
  explicit Dynamics(DynamicsModel model);
  ~Dynamics();
  Dynamics(Dynamics&&) noexcept;
  Dynamics& operator=(Dynamics&&) noexcept;
  Dynamics(const Dynamics&) = delete;
  Dynamics& operator=(const Dynamics&) = delete;

  DynamicsModel model() const { return model_; }
  const std::array<float, G1_NUM_MOTOR>& getStiffness() const { return kp_; }
  const std::array<float, G1_NUM_MOTOR>& getDamping() const { return kd_; }

  std::array<float, G1_NUM_MOTOR> getFFTau(
      const std::array<double, G1_NUM_MOTOR>& q_target) const;

 private:
  DynamicsModel model_;
  std::array<float, G1_NUM_MOTOR> kp_{};
  std::array<float, G1_NUM_MOTOR> kd_{};

 
  struct PinochioImplementation;
  std::unique_ptr<PinochioImplementation> pin_;
};
