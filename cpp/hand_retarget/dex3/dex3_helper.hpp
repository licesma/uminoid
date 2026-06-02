#pragma once

// Small, side-aware helpers shared by the dex3 controller and retargeter.
// Header-only: every helper here is a one-liner or a tight loop.

#include "hand_retarget/dex3/dex3_retarget_config.hpp"
#include "hand_retarget/dex3/dex3_values.hpp"

#include <unitree/idl/hg/HandCmd_.hpp>

#include <cstdint>

namespace dex3_helper {

// HandCmd_ motor mode-byte layout: low nibble = motor id, next 3 bits = status,
// top bit = timeout flag.
inline uint8_t make_mode_byte(uint8_t id, uint8_t status, uint8_t timeout = 0) {
    return uint8_t((id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7));
}

// True if `j` is one of the three palm-bend joints we drive from manus curl.
inline bool is_active(Dex3Joint j) {
    for (Dex3Joint a : Dex3ActiveJoints) if (a == j) return true;
    return false;
}

// DDS wire slot for `j` on the given side.
inline int wire_slot(Dex3Joint j, Dex3Side side) {
    return (side == Dex3Side::Left ? Dex3WireSlotLeft : Dex3WireSlotRight)[j];
}

// One-shot setup of a HandCmd_: resizes motor_cmd, writes mode bytes, zeros
// q/dq/tau, and assigns uniform kp/kd (from dex3_retarget_config) to every
// joint. The caller is expected to overwrite q every publish.
inline void prepare_cmd(unitree_hg::msg::dds_::HandCmd_& cmd, Dex3Side side) {
    cmd.motor_cmd().resize(Dex3NumJoints);
    for (int j = 0; j < Dex3NumJoints; ++j) {
        const auto joint = static_cast<Dex3Joint>(j);
        const int  slot  = wire_slot(joint, side);
        cmd.motor_cmd()[slot].mode(make_mode_byte(uint8_t(slot), 0x01));
        cmd.motor_cmd()[slot].q(0.0f);
        cmd.motor_cmd()[slot].dq(0.0f);
        cmd.motor_cmd()[slot].tau(0.0f);
        cmd.motor_cmd()[slot].kp(Dex3Kp);
        cmd.motor_cmd()[slot].kd(Dex3Kd);
    }
}

}  // namespace dex3_helper
