#pragma once

// Per-joint constant arrays indexed by G1JointIndex.
//
// Values:
//   - stiffness, damping : PD gains used by LowCommandWriter on every motor
//                          command publish.
//   - initial_pose       : the q_target the controller ramps to before
//                          handing control over 

#include <array>

#include "g1Enums.hpp"
#include "g1Structs.hpp"   // for G1_NUM_MOTOR

// -----------------------------------------------------------------------------
// Stiffness and Damping
// -----------------------------------------------------------------------------
//
// Legs + waist values come from third_party/AMO/play_amo.py 
// Arm + wrist values keep a low-stiffness baseline so the exo operator does
// not fight stiff motors when teleoperating. 
inline constexpr std::array<float, G1_NUM_MOTOR> stiffness = []() {
    std::array<float, G1_NUM_MOTOR> s{};
    // -- AMO-trained: legs --
    s[LeftHipPitch]    = 150.0f;
    s[LeftHipRoll]     = 150.0f;
    s[LeftHipYaw]      = 150.0f;
    s[LeftKnee]        = 300.0f;
    s[LeftAnklePitch]  =  80.0f;
    s[LeftAnkleRoll]   =  20.0f;
    s[RightHipPitch]   = 150.0f;
    s[RightHipRoll]    = 150.0f;
    s[RightHipYaw]     = 150.0f;
    s[RightKnee]       = 300.0f;
    s[RightAnklePitch] =  80.0f;
    s[RightAnkleRoll]  =  20.0f;
    // -- AMO-trained: waist --
    s[WaistYaw]        = 400.0f;
    s[WaistRoll]       = 400.0f;
    s[WaistPitch]      = 400.0f;
    // -- exo-driven: left arm + wrist --
    s[LeftShoulderPitch] = 40.0f;
    s[LeftShoulderRoll]  = 40.0f;
    s[LeftShoulderYaw]   = 40.0f;
    s[LeftElbow]         = 40.0f;
    s[LeftWristRoll]     = 40.0f;
    s[LeftWristPitch]    = 40.0f;
    s[LeftWristYaw]      = 40.0f;
    // -- exo-driven: right arm + wrist --
    s[RightShoulderPitch] = 40.0f;
    s[RightShoulderRoll]  = 40.0f;
    s[RightShoulderYaw]   = 40.0f;
    s[RightElbow]         = 40.0f;
    s[RightWristRoll]     = 40.0f;
    s[RightWristPitch]    = 40.0f;
    s[RightWristYaw]      = 40.0f;
    return s;
}();

inline constexpr std::array<float, G1_NUM_MOTOR> damping = []() {
    std::array<float, G1_NUM_MOTOR> d{};
    // -- AMO-trained: legs --
    d[LeftHipPitch]    = 2.0f;
    d[LeftHipRoll]     = 2.0f;
    d[LeftHipYaw]      = 2.0f;
    d[LeftKnee]        = 4.0f;
    d[LeftAnklePitch]  = 2.0f;
    d[LeftAnkleRoll]   = 1.0f;
    d[RightHipPitch]   = 2.0f;
    d[RightHipRoll]    = 2.0f;
    d[RightHipYaw]     = 2.0f;
    d[RightKnee]       = 4.0f;
    d[RightAnklePitch] = 2.0f;
    d[RightAnkleRoll]  = 1.0f;
    // -- AMO-trained: waist --
    d[WaistYaw]        = 15.0f;
    d[WaistRoll]       = 15.0f;
    d[WaistPitch]      = 15.0f;
    // -- exo-driven: left arm + wrist --
    d[LeftShoulderPitch] = 1.0f;
    d[LeftShoulderRoll]  = 1.0f;
    d[LeftShoulderYaw]   = 1.0f;
    d[LeftElbow]         = 1.0f;
    d[LeftWristRoll]     = 1.0f;
    d[LeftWristPitch]    = 1.0f;
    d[LeftWristYaw]      = 1.0f;
    // -- exo-driven: right arm + wrist --
    d[RightShoulderPitch] = 1.0f;
    d[RightShoulderRoll]  = 1.0f;
    d[RightShoulderYaw]   = 1.0f;
    d[RightElbow]         = 1.0f;
    d[RightWristRoll]     = 1.0f;
    d[RightWristPitch]    = 1.0f;
    d[RightWristYaw]      = 1.0f;
    return d;
}();

// -----------------------------------------------------------------------------
// Psi-0 gains
// -----------------------------------------------------------------------------
inline constexpr std::array<float, G1_NUM_MOTOR> psi0_stiffness = []() {
    std::array<float, G1_NUM_MOTOR> s = stiffness;  // legs + waist unchanged
    // -- left arm + wrist --
    s[LeftShoulderPitch] = 50.0f;
    s[LeftShoulderRoll]  = 50.0f;
    s[LeftShoulderYaw]   = 50.0f;
    s[LeftElbow]         = 50.0f;
    s[LeftWristRoll]     = 30.0f;
    s[LeftWristPitch]    = 30.0f;
    s[LeftWristYaw]      = 30.0f;
    // -- right arm + wrist --
    s[RightShoulderPitch] = 50.0f;
    s[RightShoulderRoll]  = 50.0f;
    s[RightShoulderYaw]   = 50.0f;
    s[RightElbow]         = 50.0f;
    s[RightWristRoll]     = 30.0f;
    s[RightWristPitch]    = 30.0f;
    s[RightWristYaw]      = 30.0f;
    return s;
}();

inline constexpr std::array<float, G1_NUM_MOTOR> psi0_damping = []() {
    std::array<float, G1_NUM_MOTOR> d = damping;  // legs + waist unchanged
    // -- left arm + wrist --
    d[LeftShoulderPitch] = 7.5f;
    d[LeftShoulderRoll]  = 7.5f;
    d[LeftShoulderYaw]   = 7.5f;
    d[LeftElbow]         = 7.5f;
    d[LeftWristRoll]     = 6.0f;
    d[LeftWristPitch]    = 6.0f;
    d[LeftWristYaw]      = 6.0f;
    // -- right arm + wrist --
    d[RightShoulderPitch] = 7.5f;
    d[RightShoulderRoll]  = 7.5f;
    d[RightShoulderYaw]   = 7.5f;
    d[RightElbow]         = 7.5f;
    d[RightWristRoll]     = 6.0f;
    d[RightWristPitch]    = 6.0f;
    d[RightWristYaw]      = 6.0f;
    return d;
}();

// -----------------------------------------------------------------------------
// Initial pose
// -----------------------------------------------------------------------------
//
// q_target the controller ramps to from whatever pose the robot is in at
// startup, before handing control over to AMO + the exo path.
//
// Legs + waist sit in AMO's default crouch (third_party/AMO/play_amo.py
// `default_dof_pos[:15]`)
//
// Elbows ramp to DISABLED_ARM_ELBOW_Q so the limbs don't dangle at q=0 before
// arm following kicks in (whether enabled or not). All other arm + wrist
// slots stay at 0.
inline constexpr double DISABLED_ARM_ELBOW_Q = 0.4;

inline constexpr std::array<double, G1_NUM_MOTOR> initial_pose = []() {
    std::array<double, G1_NUM_MOTOR> p{};
    // -- AMO crouch: legs --
    p[LeftHipPitch]    = -0.1;
    p[LeftKnee]        =  0.3;
    p[LeftAnklePitch]  = -0.2;
    p[RightHipPitch]   = -0.1;
    p[RightKnee]       =  0.3;
    p[RightAnklePitch] = -0.2;
    // -- waist: zeros (AMO default) --
    // -- arms: parked elbows, zeros elsewhere --
    p[LeftElbow]       = DISABLED_ARM_ELBOW_Q;
    p[RightElbow]      = DISABLED_ARM_ELBOW_Q;
    return p;
}();
