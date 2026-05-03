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
    (std::filesystem::path(__FILE__).parent_path() / "dex3_retarget_config.yaml")
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

std::string csv_header() {
    std::string s = "collection_id,host_timestamp";
    auto add_block = [&](const char* side, const char* signal) {
        for (int i = 0; i < Dex3NumJoints; ++i) {
            const auto j = static_cast<Dex3Joint>(i);
            s += std::string(",") + side + "_" + signal + "_" + Dex3JointNames[j];
        }
    };
    for (const char* signal : {"cmd", "actual", "force"}) add_block("L", signal);
    for (const char* signal : {"cmd", "actual", "force"}) add_block("R", signal);
    return s;
}

void append_side(std::string& line, std::optional<SingleDex3Retargeter>& side) {
    // Stable schema: 3 blocks of Dex3NumJoints columns (cmd, actual, force).
    // Disabled side -> "null" everywhere, matching inspire's convention.
    if (!side) {
        for (int i = 0; i < 3 * Dex3NumJoints; ++i) line += ",null";
        return;
    }
    const auto cmd_q = side->last_cmd_q();
    const auto st    = side->state();
    auto append_block = [&](const SingleDex3Retargeter::JointPose& v) {
        for (int i = 0; i < Dex3NumJoints; ++i) {
            const auto j = static_cast<Dex3Joint>(i);
            line += "," + std::to_string(v[j]);
        }
    };
    append_block(cmd_q);
    append_block(st.q);
    append_block(st.tau);
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
        append_side(line, left_);
        append_side(line, right_);
        hand_csv_.write_line(line);
    }
}

void Dex3Retargeter::finish() {
    hand_csv_.close();
}
