#include "hand_retarget/inspire/inspire_retargeter.hpp"
#include "manus/manus_hand.hpp"
#include "utils/repo_constants.hpp"

#include "utils/time.hpp"
#include "utils/type.hpp"

#include <algorithm>
#include <filesystem>
#include <optional>

namespace {

static const std::string BOUNDS_PATH =
    (std::filesystem::path(__FILE__).parent_path() / "inspire_retarget_bounds.yaml")
        .lexically_normal()
        .string();

// Column order for one hand, one signal (6 fingers).
static const char* FINGERS[6] = {
    "pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation",
};

std::string csv_header() {
    std::string s = "collection_id,host_timestamp";
    // Signals: commanded target, measured actual angle, measured fingertip force (N).
    for (const char* side : {"left", "right"})
        for (const char* signal : {"cmd", "actual", "force"})
            for (const char* finger : FINGERS)
                s += std::string(",") + side + "_" + signal + "_" + finger;
    return s;
}

std::string format_line(int collection_id,
                        int64_t timestamp,
                        const opt<InspirePose>& left_cmd,
                        const opt<InspirePose>& right_cmd,
                        const InspireFeedback& left_fb,
                        const InspireFeedback& right_fb) {
    std::string s = std::to_string(collection_id) + "," + std::to_string(timestamp);
    auto append = [&](const opt<InspirePose>& p) {
        for (int i = 0; i < 6; ++i)
            s += p ? ("," + std::to_string((*p)(i))) : ",null";
    };
    append(left_cmd);  append(left_fb.actual);  append(left_fb.force);
    append(right_cmd); append(right_fb.actual); append(right_fb.force);
    return s;
}

}  // namespace

InspireRetargeter::InspireRetargeter(
    bool left_enabled, bool right_enabled,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error
)
    : HandRetargeter(left_enabled, right_enabled, recording_label, raise_error)
{
    YAML::Node config = YAML::LoadFile(BOUNDS_PATH);
    left_bounds_  = load_bounds(config["left"]);
    right_bounds_ = load_bounds(config["right"]);

    if (!recording_label_.empty()) {
        const std::string dir = repo_constants::DATA_DIR + "/" + recording_label_;
        hand_csv_ = CsvSaver(dir + "/inspire_hand.csv", csv_header());
    }
}

double InspireRetargeter::scale(float value, double low, double high) {
    return std::clamp((value - low) / (high - low), 0.0, 1.0);
}


InspireRetargeter::HandBounds InspireRetargeter::load_bounds(const YAML::Node& node) {
    HandBounds b;
    b.index  = {node["index"]["low"].as<double>(),  node["index"]["high"].as<double>()};
    b.middle = {node["middle"]["low"].as<double>(), node["middle"]["high"].as<double>()};
    b.ring   = {node["ring"]["low"].as<double>(),   node["ring"]["high"].as<double>()};
    b.pinky  = {node["pinky"]["low"].as<double>(),  node["pinky"]["high"].as<double>()};
    b.thumb  = {node["thumb"]["low"].as<double>(),  node["thumb"]["high"].as<double>()};
    return b;
}

opt<InspirePose> InspireRetargeter::retarget(const opt<ManusHand>& hand, HandSide side) const {
    if(hand){
        const auto& bounds = (side == HandSide::LEFT) ? left_bounds_ : right_bounds_;

        InspirePose out;
        out(0) = scale(hand->pinky.wrist_to_tip,   bounds.pinky.low,  bounds.pinky.high);
        out(1) = scale(hand->ring.wrist_to_tip,    bounds.ring.low,   bounds.ring.high);
        out(2) = scale(hand->middle.wrist_to_tip,  bounds.middle.low, bounds.middle.high);
        out(3) = scale(hand->index.wrist_to_tip,   bounds.index.low,  bounds.index.high);
        out(4) = scale(hand->thumb.pinky_to_thumb, bounds.thumb.low,  bounds.thumb.high);
        out(5) = 1.0;  // thumb_rotation
        return out;
    }
   return std::nullopt;
}

void InspireRetargeter::retarget_step(
    const opt<ManusHand>& left,
    const opt<ManusHand>& right,
    int  collection_id,
    bool paused
) {
    opt<InspirePose> left_target  = left_enabled_  ? retarget(left,  HandSide::LEFT)  : std::nullopt;
    opt<InspirePose> right_target = right_enabled_ ? retarget(right, HandSide::RIGHT) : std::nullopt;

    send(left_target, right_target);

    auto [left_fb, right_fb] = read_feedback();

    if (!paused && hand_csv_) {
        hand_csv_.write_line(
            format_line(collection_id, Time::ts(),
                        left_target, right_target, left_fb, right_fb));
    }
}

void InspireRetargeter::finish() {
    hand_csv_.close();
}
