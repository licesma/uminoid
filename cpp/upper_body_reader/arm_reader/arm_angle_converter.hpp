#pragma once

#include "../constants.hpp"
#include "skeleton_arm.hpp"

#include "../../g1/model/g1Enums.hpp"
#include "../../utils/bounds_loader.hpp"
#include "../../utils/circular_math.hpp"
#include "../../utils/metadata_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#ifndef G1_BOUNDS_PATH
#define G1_BOUNDS_PATH "../g1/model/upperBodyJointBounds.yaml"
#endif

#ifndef DYNAMIXEL_BOUNDS_PATH
#define DYNAMIXEL_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
#endif

// Sentinel emitted by the exo reader for a joint with no usable reading.
inline constexpr uint16_t INVALID_EXO_READING = 5000;

inline constexpr std::array<G1JointIndex, ARM_JOINT_COUNT> LEFT_ARM_JOINTS = {
    G1JointIndex::LeftShoulderPitch, G1JointIndex::LeftShoulderRoll,
    G1JointIndex::LeftShoulderYaw,   G1JointIndex::LeftElbow,
    G1JointIndex::LeftWristRoll,     G1JointIndex::LeftWristPitch,
    G1JointIndex::LeftWristYaw,
};
inline constexpr std::array<G1JointIndex, ARM_JOINT_COUNT> RIGHT_ARM_JOINTS = {
    G1JointIndex::RightShoulderPitch, G1JointIndex::RightShoulderRoll,
    G1JointIndex::RightShoulderYaw,   G1JointIndex::RightElbow,
    G1JointIndex::RightWristRoll,     G1JointIndex::RightWristPitch,
    G1JointIndex::RightWristYaw,
};

// Converts raw exo encoder samples into G1 joint angles (radians).
// Stateless given the three calibration tables; an invalid reading -> NaN.
// This is the instantaneous mapping only (no ramp / velocity clamp).
class ArmAngleConverter {
 public:
  // Loads the exo (dynamixel) and G1 joint bounds from the fixed YAML paths;
  // these are the only calibration tables the project uses.
  ArmAngleConverter()
      : metadata_(LoadMetadata(DYNAMIXEL_BOUNDS_PATH)),
        g1_bounds_(LoadBounds(G1_BOUNDS_PATH)),
        reader_bounds_(LoadBounds(DYNAMIXEL_BOUNDS_PATH)) {}

  std::array<double, ARM_JOINT_COUNT> convert(const ArmLine& sample,
                                              bool from_left) const {
    const auto& joints = from_left ? LEFT_ARM_JOINTS : RIGHT_ARM_JOINTS;
    std::array<double, ARM_JOINT_COUNT> out{};
    for (size_t i = 0; i < ARM_JOINT_COUNT; ++i)
      out[i] = to_g1_angle(joints[i], sample.data[i]);
    return out;
  }

 private:
  double to_g1_angle(G1JointIndex joint, uint16_t bit_value) const {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    const auto [lower, upper] = reader_bounds_[joint];
    const double value =
        (bit_value - ENCODER_RESOLUTION / 2.0) * ENCODER_PRECISION_RAD;

    if (bit_value == INVALID_EXO_READING) return invalid;
    if (circularDistance(upper, lower) < circularDistance(value, lower))
      return invalid;

    const ReadingMetadata md = metadata_[joint];
    const double net_angle = md.skeleton_ref == LOWER_BOUND
                                 ? circularDistance(value, lower)
                                 : circularDistance(upper, value);

    const auto [low, high] = g1_bounds_[joint];
    const double reference = md.g1_ref == LOWER_BOUND ? low : high;
    const double direction = md.g1_ref == LOWER_BOUND ? 1.0 : -1.0;
    return std::clamp(reference + direction * net_angle, low, high);
  }

  JointsReadingMetadata metadata_{};
  JointBounds g1_bounds_{};
  JointBounds reader_bounds_{};
};
