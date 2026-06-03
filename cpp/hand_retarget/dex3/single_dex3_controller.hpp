#pragma once

#include "hand_retarget/dex3/dex3_values.hpp"

#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

/**
 * DDS-side controller for ONE Dex3-1 hand.
 *
 * Subscribes rt/dex3/{side}/state, publishes rt/dex3/{side}/cmd. All public
 * I/O is in Dex3Joint-indexed Dex3PerJoint<float>; the wire-slot mapping
 * is hidden behind Dex3WireSlot{Left,Right}.
 *
 * Does NOT call ChannelFactory::Init — the parent (Dex3Retargeter) does
 * that once for both sides. PD gains live in dex3_retarget_config.hpp;
 * the ramp rate lives in this file's .cpp; hardware facts (joint limits,
 * wire slots) come from dex3_values.hpp.
 */
class SingleDex3Controller {
public:
    using JointPose = Dex3PerJoint<float>;

    using PressArray = std::array<
        std::array<float, Dex3PressValuesPerEntry>, Dex3PressSensorCount>;

    struct State {
        JointPose  q{};
        JointPose  tau{};     // tau_est from MotorState_, fingertip-force proxy
        PressArray press{};   // press_sensor_state[i].pressure() per i
    };

    explicit SingleDex3Controller(Dex3Side side);

    bool      is_ready  () const;
    JointPose startup_q () const;
    State     state     () const;
    JointPose last_cmd_q() const;

    // Clamps each joint to URDF limits and publishes. No-op until the first
    // state message arrives. Returns the value actually written.
    JointPose send(const JointPose& target_q);

    // Same as send(), but slew-limits the change from the previous publish
    // at an internal ramp rate (seeded from startup_q on the first call).
    // Safety wrapper for use while the rest of the system is still being
    // tuned; will eventually be retired in favor of plain send().
    JointPose send_slewed(const JointPose& target_q);

private:
    void on_state(const void* m);

    int wire_slot(Dex3Joint j) const;
    Dex3Bounds bounds(Dex3Joint j) const;

    Dex3Side    side_;
    std::string cmd_topic_;
    std::string state_topic_;

    std::atomic<bool>                     startup_captured_{false};
    JointPose                             startup_q_buf_{};
    JointPose                             current_cmd_q_{};
    bool                                  seeded_ = false;
    std::chrono::steady_clock::time_point last_send_{};

    mutable std::mutex                 state_mtx_;
    unitree_hg::msg::dds_::HandState_  state_;
    unitree_hg::msg::dds_::HandCmd_    cmd_;

    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>     cmd_pub_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>> state_sub_;
};
