#pragma once
#include "../g1/model/g1Enums.hpp"
#include "../g1/model/g1Structs.hpp"
#include <array>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>


enum JointBoundType {
  LOWER_BOUND = 0,
  UPPER_BOUND = 1
};

inline JointBoundType operator!(JointBoundType ref) {
  return ref == LOWER_BOUND ? UPPER_BOUND : LOWER_BOUND;
}

struct ReadingMetadata {
  JointBoundType skeleton_ref;
  JointBoundType g1_ref;
};

struct JointsReadingMetadata {
  std::array<ReadingMetadata, G1_NUM_MOTOR> correct_bound{};

  ReadingMetadata& operator[](G1JointIndex idx) { return correct_bound[idx]; }
  const ReadingMetadata& operator[](G1JointIndex idx) const { return correct_bound[idx]; }
};

/** Load per-joint metadata from a YAML file (e.g. dynamixel_bounds.yaml). */
inline JointsReadingMetadata LoadMetadata(const std::string& path) {
  try {
    YAML::Node config = YAML::LoadFile(path);

    auto getMetadata = [&config](const char* joint) -> ReadingMetadata {
      std::string ref_val =  config[joint]["correct_boundary"].as<std::string>();
      bool align_val = config[joint]["direction_aligned"].as<bool>();
      JointBoundType skeleton_ref = ref_val == "lower" ? LOWER_BOUND : UPPER_BOUND;
      return {skeleton_ref, align_val ? skeleton_ref : !skeleton_ref };
    };

    JointsReadingMetadata meta;
    meta[LeftShoulderPitch]  = getMetadata("left_shoulder_pitch");
    meta[LeftShoulderRoll]   = getMetadata("left_shoulder_roll");
    meta[LeftShoulderYaw]    = getMetadata("left_shoulder_yaw");
    meta[LeftElbow]          = getMetadata("left_elbow");
    meta[LeftWristRoll]      = getMetadata("left_wrist_roll");
    meta[LeftWristPitch]     = getMetadata("left_wrist_pitch");
    meta[LeftWristYaw]       = getMetadata("left_wrist_yaw");
    meta[RightShoulderPitch] = getMetadata("right_shoulder_pitch");
    meta[RightShoulderRoll]  = getMetadata("right_shoulder_roll");
    meta[RightShoulderYaw]   = getMetadata("right_shoulder_yaw");
    meta[RightElbow]         = getMetadata("right_elbow");
    meta[RightWristRoll]     = getMetadata("right_wrist_roll");
    meta[RightWristPitch]    = getMetadata("right_wrist_pitch");
    meta[RightWristYaw]      = getMetadata("right_wrist_yaw");
    return meta;
  } catch (const YAML::Exception& e) {
    throw std::runtime_error(std::string("LoadMetadata: ") + path + ": " +
                            e.what());
  }
}
