#include "hand_retarget/dex3/dex3_retargeter.hpp"

#include "hand_retarget/dex3/dex3_values.hpp"
#include "manus/manus_hand.hpp"
#include "utils/repo_constants.hpp"
#include "utils/time.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

// Quiet window after creating publishers, so DDS discovery converges before
// the first tick fires. Done once at the parent (not per-side) so two sides
// don't sleep 2s.
constexpr auto kDdsDiscoverySleep = std::chrono::seconds(1);

const std::string CONFIG_PATH =
    (std::filesystem::path(__FILE__).parent_path() / "dex3_manus_bounds.yaml")
        .lexically_normal().string();

SingleDex3Retargeter::FingerBounds parse_fb(const YAML::Node& n) {
    return { n["low"].as<double>(),
             n["high"].as<double>(),
             n["invert"] ? n["invert"].as<bool>() : false };
}

SingleDex3Retargeter::ManusBounds load_manus_bounds(const YAML::Node& root, Dex3Side side) {
    const char* side_label = (side == Dex3Side::Left) ? "left" : "right";
    if (!root["bounds"] || !root["bounds"][side_label])
        throw std::runtime_error(std::string("Dex3Retargeter: missing bounds.") + side_label);
    const YAML::Node& b = root["bounds"][side_label];
    return { parse_fb(b["thumb"]), parse_fb(b["index"]), parse_fb(b["middle"]) };
}

// Per-side column ordering follows the firmware's wire layout (matching
// Humanoid-Exo-Learning's Dex3 schema). Names stay physical-joint-readable.
const std::array<Dex3Joint, Dex3NumJoints>& slot_to_joint(Dex3Side s) {
    return (s == Dex3Side::Left) ? Dex3JointAtWireSlotLeft : Dex3JointAtWireSlotRight;
}

std::string csv_header() {
    std::string s = "collection_id,host_timestamp";
    auto add_motor_block = [&](const char* side_label, Dex3Side side, const char* signal) {
        for (int slot = 0; slot < Dex3NumJoints; ++slot) {
            const auto j = slot_to_joint(side)[slot];
            s += std::string(",") + side_label + "_" + signal + "_" + Dex3JointNames[j];
        }
    };
    auto add_press_block = [&](const char* side_label) {
        for (int e = 0; e < Dex3PressSensorCount; ++e)
            for (int c = 0; c < Dex3PressValuesPerEntry; ++c)
                s += std::string(",") + side_label + "_press_e"
                     + std::to_string(e) + "_c" + std::to_string(c);
    };
    for (const auto& [label, side] : std::initializer_list<std::pair<const char*, Dex3Side>>{
            {"L", Dex3Side::Left}, {"R", Dex3Side::Right}}) {
        for (const char* signal : {"cmd", "actual", "force"}) add_motor_block(label, side, signal);
        add_press_block(label);
    }
    return s;
}

void append_side(std::string& line, std::optional<SingleDex3Retargeter>& side, Dex3Side which) {
    // Stable schema: 3 blocks of Dex3NumJoints (cmd, actual, force) +
    // Dex3PressSensorCount * Dex3PressValuesPerEntry pressure cells.
    // Disabled side -> "null" everywhere (matches inspire's convention).
    constexpr int kPressCols = Dex3PressSensorCount * Dex3PressValuesPerEntry;
    if (!side) {
        for (int i = 0; i < 3 * Dex3NumJoints + kPressCols; ++i) line += ",null";
        return;
    }
    const auto& joint_at_slot = slot_to_joint(which);
    const auto  cmd_q = side->last_cmd_q();
    const auto  st    = side->state();

    auto append_motor_block = [&](const SingleDex3Retargeter::JointPose& v) {
        for (int slot = 0; slot < Dex3NumJoints; ++slot)
            line += "," + std::to_string(v[joint_at_slot[slot]]);
    };
    append_motor_block(cmd_q);
    append_motor_block(st.q);
    append_motor_block(st.tau);
    for (int e = 0; e < Dex3PressSensorCount; ++e)
        for (int c = 0; c < Dex3PressValuesPerEntry; ++c)
            line += "," + std::to_string(st.press[e][c]);
}

}  // namespace

Dex3Retargeter::Dex3Retargeter(
    bool left_enabled, bool right_enabled,
    const std::string& network_interface,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error
)
    : HandRetargeter(left_enabled, right_enabled, recording_label, raise_error)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(CONFIG_PATH);
    } catch (const std::exception& e) {
        throw std::runtime_error("Dex3Retargeter: failed to load " + CONFIG_PATH + ": " + e.what());
    }

    // ChannelFactory is a singleton; Init is idempotent across same-iface
    // initializations. Done here once for both sides.
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    if (left_enabled)
        left_.emplace (Dex3Side::Left,  load_manus_bounds(root, Dex3Side::Left));
    if (right_enabled)
        right_.emplace(Dex3Side::Right, load_manus_bounds(root, Dex3Side::Right));

    std::this_thread::sleep_for(kDdsDiscoverySleep);

    if (!recording_label_.empty()) {
        const std::string dir = repo_constants::DATA_DIR + "/" + recording_label_;
        hand_csv_ = CsvSaver(dir + "/dex3_hand.csv", csv_header());
    }
}

void Dex3Retargeter::retarget_step(
    const opt<ManusHand>& left,
    const opt<ManusHand>& right,
    int  collection_id,
    bool paused
) {
    if (left_)  left_ ->step(left);
    if (right_) right_->step(right);

    if (!paused && hand_csv_) {
        std::string line = std::to_string(collection_id) + "," + std::to_string(Time::ts());
        append_side(line, left_,  Dex3Side::Left);
        append_side(line, right_, Dex3Side::Right);
        hand_csv_.write_line(line);
    }
}

void Dex3Retargeter::finish() {
    hand_csv_.close();
}
