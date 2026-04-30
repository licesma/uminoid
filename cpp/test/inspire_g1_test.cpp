/**
 * @file inspire_g1_test.cpp
 * @brief Test moving the index finger on the Inspire hand through the G1 robot
 *        via DDS (the same path the G1 uses in production).
 *
 * Usage:
 *   ./inspire_g1_test [network_interface]
 *
 * The inspire_g1 service must be running so that DDS commands are forwarded to
 * the physical hand over serial.
 */

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <eigen3/Eigen/Dense>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>

static std::atomic<bool> running{true};
static void signal_handler(int) { running.store(false); }

// Finger order: pinky, ring, middle, index, thumb_bend, thumb_rotation
constexpr int kIndexFinger = 3;
constexpr int kNumFingers = 6;
// Right hand occupies indices 0-5, left hand 6-11
constexpr int kRightHandOffset = 0;
constexpr int kLeftHandOffset = 6;
constexpr int kTotalHandJoints = 12;

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);

    std::string network_interface = argc > 1 ? argv[1] : "";
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    // --- Publisher: send hand commands --->  "rt/inspire/cmd" ---
    auto cmd_pub = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>(
        "rt/inspire/cmd");
    cmd_pub->InitChannel();

    unitree_go::msg::dds_::MotorCmds_ cmd;
    cmd.cmds().resize(kTotalHandJoints);

    // --- Subscriber: receive hand state <--- "rt/inspire/state" ---
    std::mutex state_mtx;
    unitree_go::msg::dds_::MotorStates_ state;
    state.states().resize(kTotalHandJoints);

    auto state_sub = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(
        "rt/inspire/state");
    state_sub->InitChannel([&](const void* message) {
        std::lock_guard<std::mutex> lock(state_mtx);
        state = *static_cast<const unitree_go::msg::dds_::MotorStates_*>(message);
    });

    std::cout << "Inspire G1 index-finger test\n";
    std::cout << "Waiting 1 s for DDS discovery...\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start with all fingers open (1.0)
    for (int i = 0; i < kTotalHandJoints; ++i)
        cmd.cmds()[i].q() = 1.0f;

    // --- Test: sweep the right-hand index finger closed then open ---
    const float step = 0.05f;
    const auto interval = std::chrono::milliseconds(100);
    float index_target = 1.0f;
    bool closing = true;

    std::cout << "Sweeping both index fingers (close -> open -> close ...)\n";
    std::cout << "Press Ctrl+C to stop.\n\n";

    while (running.load())
    {
        // Update index finger target
        if (closing) {
            index_target -= step;
            if (index_target <= 0.0f) {
                index_target = 0.0f;
                closing = false;
            }
        } else {
            index_target += step;
            if (index_target >= 1.0f) {
                index_target = 1.0f;
                closing = true;
            }
        }

        cmd.cmds()[kRightHandOffset + kIndexFinger].q() = index_target;
        cmd.cmds()[kLeftHandOffset + kIndexFinger].q() = index_target;
        cmd_pub->Write(cmd);

        // Read back state
        {
            std::lock_guard<std::mutex> lock(state_mtx);
            float actual_r = state.states()[kRightHandOffset + kIndexFinger].q();
            float actual_l = state.states()[kLeftHandOffset + kIndexFinger].q();
            std::cout << "target: " << index_target
                      << "  R actual: " << actual_r
                      << "  L actual: " << actual_l << "        \r" << std::flush;
        }

        std::this_thread::sleep_for(interval);
    }

    // Return to open position before exiting
    for (int i = 0; i < kTotalHandJoints; ++i)
        cmd.cmds()[i].q() = 1.0f;
    cmd_pub->Write(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "\nDone. Hand returned to open position.\n";
    return 0;
}
