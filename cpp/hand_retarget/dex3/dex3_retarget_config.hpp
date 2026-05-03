#pragma once

// Retarget-side decisions for the Dex3-1 hand. These are choices we make,
// not hardware facts (those live in dex3_values.hpp): which joints we drive
// from manus, which way they curl, and where to hold the rest.

#include "hand_retarget/dex3/dex3_values.hpp"

#include <array>

// Which joints we drive from manus curl. Everything else is static (held).
inline constexpr std::array<Dex3Joint, 3> Dex3ActiveJoints = {
    Dex3Joint::thumb_palm_bend,
    Dex3Joint::index_palm_bend,
    Dex3Joint::middle_palm_bend,
};

// For each active joint, which end of the URDF bound corresponds to curl=1
// (fully closed). The opposite end is curl=0 (fully open). Encodes motor-
// wiring polarity, which differs per side.
//
// Entries for static joints are unused (their value is taken from
// Dex3StaticHoldQ* below).
enum class Dex3CurlEnd { low, high };

inline constexpr Dex3PerJoint<Dex3CurlEnd> Dex3ActiveCurlEndLeft = {{{
    Dex3CurlEnd::low,   // thumb_rotation    (static — unused)
    Dex3CurlEnd::low,   // thumb_palm_bend   curl=1 -> bounds.low (-0.724)
    Dex3CurlEnd::low,   // thumb_tip_bend    (static — unused)
    Dex3CurlEnd::low,   // index_palm_bend   curl=1 -> bounds.low (-1.57)
    Dex3CurlEnd::low,   // index_tip_bend    (static — unused)
    Dex3CurlEnd::low,   // middle_palm_bend  curl=1 -> bounds.low (-1.57)
    Dex3CurlEnd::low,   // middle_tip_bend   (static — unused)
}}};

inline constexpr Dex3PerJoint<Dex3CurlEnd> Dex3ActiveCurlEndRight = {{{
    Dex3CurlEnd::low,   // thumb_rotation    (static — unused)
    Dex3CurlEnd::low,   // thumb_palm_bend   curl=1 -> bounds.low (-1.05)
    Dex3CurlEnd::low,   // thumb_tip_bend    (static — unused)
    Dex3CurlEnd::high,  // index_palm_bend   curl=1 -> bounds.high (1.57)
    Dex3CurlEnd::high,  // index_tip_bend    (static — unused)
    Dex3CurlEnd::high,  // middle_palm_bend  curl=1 -> bounds.high (1.57)
    Dex3CurlEnd::high,  // middle_tip_bend   (static — unused)
}}};

// Hold q for each static joint, per side. Anywhere inside the URDF bound is
// fine. Entries for active joints are unused.
inline constexpr Dex3PerJoint<float> Dex3StaticHoldQLeft = {{{
    -0.474f,   // thumb_rotation
     0.0f,     // thumb_palm_bend   (active — unused)
     0.522f,   // thumb_tip_bend
     0.0f,     // index_palm_bend   (active — unused)
    -0.628f,   // index_tip_bend
     0.0f,     // middle_palm_bend  (active — unused)
    -0.628f,   // middle_tip_bend
}}};

inline constexpr Dex3PerJoint<float> Dex3StaticHoldQRight = {{{
    -0.401f,   // thumb_rotation
     0.0f,     // thumb_palm_bend   (active — unused)
    -0.522f,   // thumb_tip_bend
     0.0f,     // index_palm_bend   (active — unused)
     0.628f,   // index_tip_bend
     0.0f,     // middle_palm_bend  (active — unused)
     0.628f,   // middle_tip_bend
}}};
