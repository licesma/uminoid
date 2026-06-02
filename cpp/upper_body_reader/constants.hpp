#pragma once

/*
################################################################################
#  ⚠️  KEEP IN SYNC  ⚠️
#  This file MUST be kept in sync with: ../../joint_reader_constants.py
################################################################################
*/

#include <cmath>
#include <array>
#include <string>

constexpr int ARM_JOINT_COUNT = 7;
constexpr int JOINT_COUNT = 14;
constexpr int ENCODER_RESOLUTION = 4096;
constexpr double ENCODER_PRECISION = 1.0 / ENCODER_RESOLUTION;
constexpr double ENCODER_PRECISION_RAD = ENCODER_PRECISION * 2.0 * M_PI;



/**
 * Encoder index in the CSV stream sent over serial.
 * Order matches JOINT_NAMES in joints.py:
 *   left arm (shoulder_pitch → wrist_yaw), then right arm.
 */
 enum class ExoIndex : int {
    LeftShoulderPitch  = 0,
    LeftShoulderRoll   = 1,
    LeftShoulderYaw    = 2,
    LeftElbow          = 3,
    LeftWristRoll      = 4,
    LeftWristPitch     = 5,
    LeftWristYaw       = 6,
    RightShoulderPitch = 7,
    RightShoulderRoll  = 8,
    RightShoulderYaw   = 9,
    RightElbow         = 10,
    RightWristRoll     = 11,
    RightWristPitch    = 12,
    RightWristYaw      = 13,
  };

  const std::array<ExoIndex, JOINT_COUNT> EXO_JOINT_INDICES = {
    ExoIndex::LeftShoulderPitch,
    ExoIndex::LeftShoulderRoll, 
    ExoIndex::LeftShoulderYaw, 
    ExoIndex::LeftElbow, 
    ExoIndex::LeftWristRoll, 
    ExoIndex::LeftWristPitch, 
    ExoIndex::LeftWristYaw, 
    ExoIndex::RightShoulderPitch, 
    ExoIndex::RightShoulderRoll, 
    ExoIndex::RightShoulderYaw, 
    ExoIndex::RightElbow, 
    ExoIndex::RightWristRoll, 
    ExoIndex::RightWristPitch,
    ExoIndex::RightWristYaw,
  };

// Bare arm joint names, in encoder order. MUST match ARM_JOINTS in py/joints.py.
inline constexpr std::array<const char*, ARM_JOINT_COUNT> ARM_JOINT_NAMES = {
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
    "wrist_roll",     "wrist_pitch",   "wrist_yaw",
};

// Header for an angle CSV (measured/command): named per-joint columns matching
// the Python side, e.g. "left_shoulder_pitch_joint".
inline std::string arm_csv_header(bool from_left) {
  const std::string side = from_left ? "left_" : "right_";
  std::string h = "collection_id,timestamp,host_timestamp";
  for (const auto* name : ARM_JOINT_NAMES) h += "," + side + name + "_joint";
  return h;
}

// Header for the raw bit CSV (arm.csv): generic joint_0..joint_N columns, since
// the values are encoder bits, not joint angles.
inline std::string raw_arm_csv_header() {
  std::string h = "collection_id,timestamp,host_timestamp";
  for (int i = 0; i < ARM_JOINT_COUNT; ++i) h += ",joint_" + std::to_string(i);
  return h;
}
  