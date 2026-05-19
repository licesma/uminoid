#include "hand_retarget/dex3/single_dex3_controller.hpp"

#include "hand_retarget/dex3/dex3_helper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

// ---- Tunables (chosen, not hardware-defined) ----
//
// Gains for the active palm-bend joints (Dex3ActiveJoints) — the ones we
// actually drive from manus curls.
constexpr float kActiveKp = 1.5f;
constexpr float kActiveKd = 0.2f;
//
// Gains for the static joints. Lower kp + relatively higher kd resists
// drift but doesn't fight tendon coupling when the palm bend moves.
constexpr float kStaticKp = 0.6f;
constexpr float kStaticKd = 0.3f;
//
// Slew limit on commanded q (rad/s), per joint per send_slewed().
constexpr float kRampRate = 0.6f;
//
// Default dt when the time delta from the previous send is non-positive.
constexpr float kDefaultDtSeconds = 0.01f;

}  // namespace

SingleDex3Controller::SingleDex3Controller(Dex3Side side) : side_(side) {
    cmd_topic_   = (side == Dex3Side::Left) ? "rt/dex3/left/cmd"   : "rt/dex3/right/cmd";
    state_topic_ = (side == Dex3Side::Left) ? "rt/dex3/left/state" : "rt/dex3/right/state";

    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        state_.motor_state().resize(Dex3NumJoints);
        state_.press_sensor_state().resize(Dex3PressSensorCount);
    }

    state_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>(state_topic_);
    state_sub_->InitChannel([this](const void* m) { on_state(m); }, 1);

    cmd_pub_ = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>(cmd_topic_);
    cmd_pub_->InitChannel();

    dex3_helper::prepare_cmd(cmd_, side_, kActiveKp, kActiveKd, kStaticKp, kStaticKd);
}

int SingleDex3Controller::wire_slot(Dex3Joint j) const {
    return (side_ == Dex3Side::Left ? Dex3WireSlotLeft : Dex3WireSlotRight)[j];
}

Dex3Bounds SingleDex3Controller::bounds(Dex3Joint j) const {
    return (side_ == Dex3Side::Left ? Dex3BoundsLeft : Dex3BoundsRight)[j];
}

void SingleDex3Controller::on_state(const void* message) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    state_ = *static_cast<const unitree_hg::msg::dds_::HandState_*>(message);
    if (!startup_captured_.load()) {
        for (int j = 0; j < Dex3NumJoints; ++j) {
            const auto joint = static_cast<Dex3Joint>(j);
            startup_q_buf_[joint] = state_.motor_state()[wire_slot(joint)].q();
        }
        startup_captured_.store(true);
    }
}

bool SingleDex3Controller::is_ready() const {
    return startup_captured_.load();
}

SingleDex3Controller::JointPose SingleDex3Controller::startup_q() const {
    return startup_q_buf_;
}

SingleDex3Controller::State SingleDex3Controller::state() const {
    State out;
    std::lock_guard<std::mutex> lock(state_mtx_);
    for (int j = 0; j < Dex3NumJoints; ++j) {
        const auto joint = static_cast<Dex3Joint>(j);
        const int  slot  = wire_slot(joint);
        out.q  [joint] = state_.motor_state()[slot].q();
        out.tau[joint] = state_.motor_state()[slot].tau_est();
    }
    for (int i = 0; i < Dex3PressSensorCount; ++i)
        out.press[i] = state_.press_sensor_state()[i].pressure();
    return out;
}

SingleDex3Controller::JointPose SingleDex3Controller::last_cmd_q() const {
    return current_cmd_q_;
}

SingleDex3Controller::JointPose SingleDex3Controller::send(const JointPose& target_q) {
    if (!startup_captured_.load()) return current_cmd_q_;

    for (int j = 0; j < Dex3NumJoints; ++j) {
        const auto       joint = static_cast<Dex3Joint>(j);
        const Dex3Bounds b     = bounds(joint);
        current_cmd_q_[joint]  = std::clamp(target_q[joint], b.low, b.high);
        cmd_.motor_cmd()[wire_slot(joint)].q(current_cmd_q_[joint]);
    }
    cmd_pub_->Write(cmd_);
    return current_cmd_q_;
}

SingleDex3Controller::JointPose SingleDex3Controller::send_slewed(const JointPose& target_q) {
    if (!startup_captured_.load()) return current_cmd_q_;

    const auto now = std::chrono::steady_clock::now();
    float dt_s = kDefaultDtSeconds;
    if (seeded_) {
        dt_s = std::chrono::duration<float>(now - last_send_).count();
        if (dt_s <= 0.0f) dt_s = kDefaultDtSeconds;
    } else {
        current_cmd_q_ = startup_q_buf_;
        seeded_ = true;
    }
    last_send_ = now;

    // Slew toward the clamped target by at most kRampRate * dt_s per joint,
    // then hand off to send() to do the final clamp + DDS publish.
    const float max_step = kRampRate * dt_s;
    JointPose   slewed{};
    for (int j = 0; j < Dex3NumJoints; ++j) {
        const auto       joint = static_cast<Dex3Joint>(j);
        const Dex3Bounds b     = bounds(joint);
        const float      clamped = std::clamp(target_q[joint], b.low, b.high);
        const float      err     = clamped - current_cmd_q_[joint];
        if (std::fabs(err) <= max_step) {
            slewed[joint] = clamped;
        } else {
            slewed[joint] = current_cmd_q_[joint] + (err > 0 ? +max_step : -max_step);
        }
    }
    return send(slewed);
}
