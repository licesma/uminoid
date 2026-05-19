#pragma once

// Hardware-defined facts about the Unitree Dex3-1 hand. From the URDF and
// firmware wiring — not tuning choices. Tunables (gains, slew limits, DDS
// discovery delays) live next to the controller, not here.

#include <array>
#include <cstddef>

// Number of bend/rotation joints per hand.
inline constexpr int Dex3NumJoints = 7;

// Number of pressure sensor entries per hand (size of press_sensor_state).
inline constexpr int Dex3PressSensorCount = 9;

// Per-entry pressure cell count (the IDL stores a fixed-size 12-float array
// of pressure values inside each PressSensorState_).
inline constexpr int Dex3PressValuesPerEntry = 12;

// Which physical hand we're talking about.
enum class Dex3Side { Left, Right };

// Physical roles of the seven joints in one hand. Indexes a Dex3PerJoint.
//
// Mapping to the Unitree wire names (motor_state[i] / motor_cmd[i]) differs
// per side and lives in Dex3WireSlot{Left,Right}.
enum class Dex3Joint : int {
    thumb_rotation,    // wire: thumb_0  — abduction across the palm
    thumb_palm_bend,   // wire: thumb_1  — active curl (MCP)
    thumb_tip_bend,    // wire: thumb_2  — distal flex
    index_palm_bend,   // wire: index_0  — active curl
    index_tip_bend,    // wire: index_1
    middle_palm_bend,  // wire: middle_0 — active curl
    middle_tip_bend,   // wire: middle_1
};

// Tiny wrapper so callers can write `Dex3Bounds[Dex3Joint::thumb_palm_bend]`
// instead of casting the enum to size_t every time.
template <typename T>
struct Dex3PerJoint {
    std::array<T, Dex3NumJoints> data;
    constexpr T&       operator[](Dex3Joint j)       { return data[static_cast<size_t>(j)]; }
    constexpr const T& operator[](Dex3Joint j) const { return data[static_cast<size_t>(j)]; }
};

struct Dex3Bounds { float low, high; };

// URDF joint limits (rad), per side, indexed by Dex3Joint.
inline constexpr Dex3PerJoint<Dex3Bounds> Dex3BoundsLeft = {{{
    {-1.05f,  1.05f },   // thumb_rotation
    {-0.724f, 1.05f },   // thumb_palm_bend
    { 0.0f,   1.75f },   // thumb_tip_bend
    {-1.57f,  0.0f  },   // index_palm_bend
    {-1.75f,  0.0f  },   // index_tip_bend
    {-1.57f,  0.0f  },   // middle_palm_bend
    {-1.75f,  0.0f  },   // middle_tip_bend
}}};

inline constexpr Dex3PerJoint<Dex3Bounds> Dex3BoundsRight = {{{
    {-1.05f,  1.05f },   // thumb_rotation
    {-1.05f,  0.742f},   // thumb_palm_bend
    {-1.75f,  0.0f  },   // thumb_tip_bend
    { 0.0f,   1.57f },   // index_palm_bend
    { 0.0f,   1.75f },   // index_tip_bend
    { 0.0f,   1.57f },   // middle_palm_bend
    { 0.0f,   1.75f },   // middle_tip_bend
}}};

// Per-side translation: Dex3Joint -> wire slot in HandState_/HandCmd_.
//
// The dex3 firmware orders left and right motors differently:
//   L wire order: thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1
//   R wire order: thumb_0, thumb_1, thumb_2, index_0,  index_1,  middle_0, middle_1
// This table is the only place in the codebase that needs to know that.
inline constexpr Dex3PerJoint<int> Dex3WireSlotLeft  = {{ 0, 1, 2, 5, 6, 3, 4 }};
inline constexpr Dex3PerJoint<int> Dex3WireSlotRight = {{ 0, 1, 2, 3, 4, 5, 6 }};

// Inverse lookup of Dex3WireSlot*: given a wire slot 0..6, what physical
// Dex3Joint sits there? Used by csv emission so the column layout matches
// the firmware's motor ordering (Humanoid-Exo-Learning's wire layout).
inline constexpr std::array<Dex3Joint, Dex3NumJoints> Dex3JointAtWireSlotLeft = {
    Dex3Joint::thumb_rotation,    // L slot 0 (thumb_0)
    Dex3Joint::thumb_palm_bend,   // L slot 1 (thumb_1)
    Dex3Joint::thumb_tip_bend,    // L slot 2 (thumb_2)
    Dex3Joint::middle_palm_bend,  // L slot 3 (middle_0)
    Dex3Joint::middle_tip_bend,   // L slot 4 (middle_1)
    Dex3Joint::index_palm_bend,   // L slot 5 (index_0)
    Dex3Joint::index_tip_bend,    // L slot 6 (index_1)
};
inline constexpr std::array<Dex3Joint, Dex3NumJoints> Dex3JointAtWireSlotRight = {
    Dex3Joint::thumb_rotation,    // R slot 0 (thumb_0)
    Dex3Joint::thumb_palm_bend,   // R slot 1 (thumb_1)
    Dex3Joint::thumb_tip_bend,    // R slot 2 (thumb_2)
    Dex3Joint::index_palm_bend,   // R slot 3 (index_0)
    Dex3Joint::index_tip_bend,    // R slot 4 (index_1)
    Dex3Joint::middle_palm_bend,  // R slot 5 (middle_0)
    Dex3Joint::middle_tip_bend,   // R slot 6 (middle_1)
};

// Physical names for the seven joints. Used for csv columns and yaml keys.
inline constexpr Dex3PerJoint<const char*> Dex3JointNames = {{
    "thumb_rotation",
    "thumb_palm_bend",
    "thumb_tip_bend",
    "index_palm_bend",
    "index_tip_bend",
    "middle_palm_bend",
    "middle_tip_bend",
}};
