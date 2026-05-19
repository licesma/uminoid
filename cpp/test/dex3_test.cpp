// Dex3-1 staged safety test. See dex3_test_config.yaml for mode selection:
//
//   Step 1 (READ):  both sides disabled, hold=false  -> subscribe only.
//   Step 2 (HOLD):  both sides disabled, hold=true   -> low-gain hold at startup q.
//   Step 3 (DRIVE): a side enabled                   -> 3-DoF curl from manus
//                                                       (or 0 if manus disabled);
//                                                       static joints held at
//                                                       targets.<joint>.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "manus/manus_reader.hpp"
#include "utils/csv_saver.hpp"
#include "utils/repo_constants.hpp"
#include "utils/time.hpp"

namespace {

constexpr int kMotorCount  = 7;
constexpr int kSensorCount = 9;

// URDF limits (rad).
constexpr float kMaxLeft [kMotorCount] = { 1.05f,  1.05f,  1.75f,  0.0f,    0.0f,    0.0f,    0.0f   };
constexpr float kMinLeft [kMotorCount] = {-1.05f, -0.724f, 0.0f,  -1.57f,  -1.75f,  -1.57f,  -1.75f };
constexpr float kMaxRight[kMotorCount] = { 1.05f,  0.742f, 0.0f,   1.57f,   1.75f,   1.57f,   1.75f };
constexpr float kMinRight[kMotorCount] = {-1.05f, -1.05f, -1.75f,  0.0f,    0.0f,    0.0f,    0.0f   };

const char* const kJointNamesLeft [kMotorCount] = {
    "thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1"};
const char* const kJointNamesRight[kMotorCount] = {
    "thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1"};

// Active palm-bend motor index per finger, per side.
struct PalmBendIndex { int thumb, index, middle; };
constexpr PalmBendIndex kPalmLeft  = {1, 5, 3};   // L: thumb_1=1, index_0=5, middle_0=3
constexpr PalmBendIndex kPalmRight = {1, 3, 5};   // R: thumb_1=1, index_0=3, middle_0=5

constexpr float kHoldKp = 0.3f;
constexpr float kHoldKd = 0.1f;

enum class Mode { Read, Hold, Drive };
const char* mode_str(Mode m) {
    switch (m) { case Mode::Read: return "READ"; case Mode::Hold: return "HOLD"; case Mode::Drive: return "DRIVE"; }
    return "?";
}

uint8_t make_mode_byte(uint8_t id, uint8_t status, uint8_t timeout = 0) {
    return uint8_t((id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7));
}

float scale_curl(float measured, float low, float high) {
    if (high <= low) return 0.0f;
    return std::clamp((measured - low) / (high - low), 0.0f, 1.0f);
}

struct FingerBounds { float low, high; bool invert; };
struct HandManusBounds { FingerBounds thumb, index, middle; };

float apply_curl(const FingerBounds& b, float measured) {
    const float c = scale_curl(measured, b.low, b.high);
    return b.invert ? (1.0f - c) : c;
}

struct HandIO {
    std::string side;       // "L" or "R"
    Mode mode = Mode::Read;
    float ramp_rate = 0.3f;
    float active_kp = 1.5f, active_kd = 0.2f;
    float static_kp = 0.6f, static_kd = 0.3f;
    std::string cmd_topic, state_topic;

    std::unique_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>> cmd_pub;
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>> state_sub;

    std::mutex state_mutex;
    unitree_hg::msg::dds_::HandState_ state;
    std::atomic<uint64_t> state_msgs{0};

    std::array<float, kMotorCount> startup_q{};
    std::atomic<bool> startup_captured{false};

    // Targets from yaml. For static joints this is the held value; for active
    // (palm-bend) joints it's the q at curl=0 (open).
    std::array<float, kMotorCount> open_q{};
    std::array<bool,  kMotorCount> is_active_joint{};   // palm bend?
    std::array<bool,  kMotorCount> target_clamped{};

    // Active joint q at curl=1.
    std::array<float, kMotorCount> closed_q{};

    // Most recent curl per finger from manus, or 0 if manus disabled.
    std::atomic<float> curl_thumb{0.0f}, curl_index{0.0f}, curl_middle{0.0f};
    HandManusBounds manus_bounds{};

    // Per-tick state for the publish loop.
    std::array<float, kMotorCount> current_cmd_q{};

    unitree_hg::msg::dds_::HandCmd_ cmd;

    const float* motor_min() const { return (side == "L") ? kMinLeft  : kMinRight; }
    const float* motor_max() const { return (side == "L") ? kMaxLeft  : kMaxRight; }
    const char*  motor_name(int i) const {
        return (side == "L") ? kJointNamesLeft[i] : kJointNamesRight[i];
    }
    PalmBendIndex palm_idx() const {
        return (side == "L") ? kPalmLeft : kPalmRight;
    }

    void on_state(const void* m) {
        std::lock_guard<std::mutex> lock(state_mutex);
        state = *static_cast<const unitree_hg::msg::dds_::HandState_*>(m);
        if (!startup_captured.load()) {
            for (int i = 0; i < kMotorCount; ++i)
                startup_q[i] = state.motor_state()[i].q();
            startup_captured.store(true);
        }
        state_msgs.fetch_add(1);
    }

    // For the given motor, what is its target q this tick?
    float target_q_for(int i) const {
        if (!is_active_joint[i]) return open_q[i];
        // Active palm-bend: linear interp open->closed by curl.
        float curl = 0.0f;
        const auto p = palm_idx();
        if      (i == p.thumb)  curl = curl_thumb.load();
        else if (i == p.index)  curl = curl_index.load();
        else if (i == p.middle) curl = curl_middle.load();
        // curl=0 -> closed (c-value),  curl=1 -> open. Matches natural
        // retargeting: real fingers curled -> robot curled.
        const float q = closed_q[i] + curl * (open_q[i] - closed_q[i]);
        return std::clamp(q, motor_min()[i], motor_max()[i]);
    }
};

void prepare_drive_cmd(HandIO& h) {
    // Seed cmd with startup_q. Per-tick we ramp current_cmd_q toward
    // target_q_for(i) and write it into the cmd message.
    h.cmd.motor_cmd().resize(kMotorCount);
    for (int i = 0; i < kMotorCount; ++i) {
        h.current_cmd_q[i] = h.startup_q[i];
        h.cmd.motor_cmd()[i].mode(make_mode_byte(uint8_t(i), 0x01));
        h.cmd.motor_cmd()[i].q(h.startup_q[i]);
        h.cmd.motor_cmd()[i].dq(0.0f);
        h.cmd.motor_cmd()[i].tau(0.0f);
        const bool active = h.is_active_joint[i];
        h.cmd.motor_cmd()[i].kp(active ? h.active_kp : h.static_kp);
        h.cmd.motor_cmd()[i].kd(active ? h.active_kd : h.static_kd);
    }
}

void prepare_hold_cmd(HandIO& h) {
    h.cmd.motor_cmd().resize(kMotorCount);
    for (int i = 0; i < kMotorCount; ++i) {
        h.cmd.motor_cmd()[i].mode(make_mode_byte(uint8_t(i), 0x01));
        h.cmd.motor_cmd()[i].q(h.startup_q[i]);
        h.cmd.motor_cmd()[i].dq(0.0f);
        h.cmd.motor_cmd()[i].tau(0.0f);
        h.cmd.motor_cmd()[i].kp(kHoldKp);
        h.cmd.motor_cmd()[i].kd(kHoldKd);
    }
}

void step_drive_ramp(HandIO& h, float dt_s) {
    const float max_step = h.ramp_rate * dt_s;
    for (int i = 0; i < kMotorCount; ++i) {
        const float target = h.target_q_for(i);
        const float err = target - h.current_cmd_q[i];
        if (std::fabs(err) <= max_step) {
            h.current_cmd_q[i] = target;
        } else {
            h.current_cmd_q[i] += (err > 0 ? +max_step : -max_step);
        }
        h.cmd.motor_cmd()[i].q(h.current_cmd_q[i]);
    }
}

std::string fmt_q(const unitree_hg::msg::dds_::HandState_& s) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    for (int i = 0; i < kMotorCount; ++i) {
        os << (i ? "  " : "") << std::setw(6) << s.motor_state()[i].q();
    }
    return os.str();
}

FingerBounds parse_fb(const YAML::Node& n) {
    return { n["low"].as<float>(),
             n["high"].as<float>(),
             n["invert"] ? n["invert"].as<bool>() : false };
}

}  // namespace

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2)
        ? argv[1]
        : (std::filesystem::path(__FILE__).parent_path() / "dex3_test_config.yaml").string();

    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config '" << config_path << "': " << e.what() << "\n";
        return 1;
    }

    const std::string iface     = yaml["network_interface"].as<std::string>();
    const bool        hold      = yaml["hold"]      ? yaml["hold"].as<bool>()       : false;
    const float       active_kp = yaml["active_kp"] ? yaml["active_kp"].as<float>() : 1.5f;
    const float       active_kd = yaml["active_kd"] ? yaml["active_kd"].as<float>() : 0.2f;
    const float       static_kp = yaml["static_kp"] ? yaml["static_kp"].as<float>() : 0.6f;
    const float       static_kd = yaml["static_kd"] ? yaml["static_kd"].as<float>() : 0.3f;
    const float       ramp_rate = yaml["ramp_rate"] ? yaml["ramp_rate"].as<float>() : 0.3f;
    const int         publish_rate_hz = yaml["publish_rate_hz"] ? yaml["publish_rate_hz"].as<int>() : 100;
    const int         print_rate_hz   = yaml["print_rate_hz"]   ? yaml["print_rate_hz"].as<int>()   : 10;
    if (ramp_rate <= 0.0f) { std::cerr << "ramp_rate must be > 0\n"; return 1; }

    // Manus block (optional).
    const YAML::Node mnode      = yaml["manus"];
    const bool       manus_on   = mnode && mnode["enabled"].as<bool>();
    const std::string m_left_addr  = manus_on ? mnode["left_address"].as<std::string>()  : "";
    const std::string m_right_addr = manus_on ? mnode["right_address"].as<std::string>() : "";
    HandManusBounds left_bounds{}, right_bounds{};
    if (manus_on) {
        const auto& b = mnode["bounds"];
        left_bounds  = { parse_fb(b["left"]["thumb"]),  parse_fb(b["left"]["index"]),  parse_fb(b["left"]["middle"]) };
        right_bounds = { parse_fb(b["right"]["thumb"]), parse_fb(b["right"]["index"]), parse_fb(b["right"]["middle"]) };
    }

    auto resolve_side = [&](const std::string& side, const YAML::Node& cfg) {
        auto h = std::make_unique<HandIO>();
        h->side = side;
        h->cmd_topic   = (side == "L") ? "rt/dex3/left/cmd"   : "rt/dex3/right/cmd";
        h->state_topic = (side == "L") ? "rt/dex3/left/state" : "rt/dex3/right/state";
        h->ramp_rate = ramp_rate;
        h->active_kp = active_kp; h->active_kd = active_kd;
        h->static_kp = static_kp; h->static_kd = static_kd;
        h->manus_bounds = (side == "L") ? left_bounds : right_bounds;

        const PalmBendIndex p = h->palm_idx();
        h->is_active_joint.fill(false);
        h->is_active_joint[p.thumb] = h->is_active_joint[p.index] = h->is_active_joint[p.middle] = true;

        // Parse targets + closed unconditionally so the would-be cmd can be
        // shown even when the hand is disabled (dry-run safety preview).
        const YAML::Node& tnode = cfg ? cfg["targets"] : YAML::Node();
        const YAML::Node& cnode = cfg ? cfg["closed"]  : YAML::Node();
        if (!tnode || !cnode) {
            std::cerr << side << " requires targets and closed maps\n"; std::exit(1);
        }
        for (const char* fname : {"thumb", "index", "middle"})
            if (!cnode[fname]) {
                std::cerr << side << " missing closed entry '" << fname << "'\n"; std::exit(1);
            }
        const char* const* names = (side == "L") ? kJointNamesLeft : kJointNamesRight;
        for (int i = 0; i < kMotorCount; ++i) {
            if (!tnode[names[i]]) {
                std::cerr << side << " targets missing '" << names[i] << "'\n"; std::exit(1);
            }
            const float requested = tnode[names[i]].as<float>();
            const float clamped   = std::clamp(requested, h->motor_min()[i], h->motor_max()[i]);
            h->open_q[i]         = clamped;
            h->target_clamped[i] = (clamped != requested);
        }
        auto set_closed = [&](int idx, const char* fname) {
            const float requested = cnode[fname].as<float>();
            h->closed_q[idx] = std::clamp(requested, h->motor_min()[idx], h->motor_max()[idx]);
            if (h->closed_q[idx] != requested) h->target_clamped[idx] = true;
        };
        set_closed(p.thumb,  "thumb");
        set_closed(p.index,  "index");
        set_closed(p.middle, "middle");

        const bool drive_enabled = cfg && cfg["enabled"].as<bool>();
        if (drive_enabled)  h->mode = Mode::Drive;
        else if (hold)      h->mode = Mode::Hold;
        else                h->mode = Mode::Read;
        return h;
    };

    std::vector<std::unique_ptr<HandIO>> hands;
    hands.push_back(resolve_side("L", yaml["left"]));
    hands.push_back(resolve_side("R", yaml["right"]));

    // ---- Banner ----
    std::cout << "================================================================\n";
    std::cout << " Dex3 test\n";
    std::cout << "   config:    " << config_path << "\n";
    std::cout << "   iface:     " << iface << "\n";
    std::cout << "   hold flag: " << (hold ? "true" : "false") << "\n";
    std::cout << "   ramp_rate: " << ramp_rate << " rad/s\n";
    std::cout << "   gains:     active kp/kd=" << active_kp << "/" << active_kd
              << "   static kp/kd=" << static_kp << "/" << static_kd << "\n";
    std::cout << "   manus:     " << (manus_on ? "ENABLED" : "disabled");
    if (manus_on) std::cout << "  (" << m_left_addr << " / " << m_right_addr << ")";
    std::cout << "\n";
    for (auto& h : hands) {
        std::cout << "   " << h->side << ": mode=" << mode_str(h->mode)
                  << "  (" << h->state_topic << " / " << h->cmd_topic << ")\n";
    }
    std::cout << "================================================================\n";

    // ---- DDS init ----
    auto raise_error = [](const std::string& m){ std::cerr << "[error] " << m << "\n"; std::exit(3); };
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);

    // ---- Manus reader (optional) ----
    std::unique_ptr<ManusReader> manus;
    std::atomic<bool> running{true};
    std::thread manus_thread;
    if (manus_on) {
        manus = std::make_unique<ManusReader>(m_left_addr, m_right_addr, raise_error);
        // Pump curls into hand atomics.
        manus_thread = std::thread([&]{
            while (running.load()) {
                auto pose = manus->wait_for_next([&]{ return !running.load(); });
                if (!pose) break;
                auto& [lh, rh] = *pose;
                for (auto& h : hands) {
                    const auto& mh = (h->side == "L") ? lh : rh;
                    if (!mh) continue;
                    const auto& b = h->manus_bounds;
                    // Thumb uses pinky_to_thumb (abduction across the palm).
                    // Index / middle use wrist_to_tip (per-finger self-curl).
                    // Each finger's bounds may set invert: true to flip 0<->1
                    // after normalization.
                    h->curl_thumb.store (apply_curl(b.thumb,  mh->thumb.pinky_to_thumb));
                    h->curl_index.store (apply_curl(b.index,  mh->index.wrist_to_tip));
                    h->curl_middle.store(apply_curl(b.middle, mh->middle.wrist_to_tip));
                }
            }
        });
    }

    for (auto& h : hands) {
        h->state_sub = std::make_unique<
            unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>(h->state_topic);
        HandIO* hp = h.get();
        h->state_sub->InitChannel([hp](const void* m) { hp->on_state(m); }, 1);
        {
            std::lock_guard<std::mutex> lock(h->state_mutex);
            h->state.motor_state().resize(kMotorCount);
            h->state.press_sensor_state().resize(kSensorCount);
        }
        if (h->mode != Mode::Read) {
            h->cmd_pub = std::make_unique<
                unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>(h->cmd_topic);
            h->cmd_pub->InitChannel();
        }
    }

    // ---- Wait for state ----
    std::cout << "Waiting up to 5s for state messages...\n";
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        bool all = true;
        for (auto& h : hands) all &= h->startup_captured.load();
        if (all) break;
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) {
            std::cerr << "Timeout: hand service running on G1?\n";
            running.store(false); if (manus_thread.joinable()) manus_thread.join();
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "State pipeline OK.\n";
    for (auto& h : hands) {
        std::cout << "   " << h->side << " startup q =";
        for (int i = 0; i < kMotorCount; ++i) std::cout << "  " << h->startup_q[i];
        std::cout << "\n";
    }

    // ---- Build cmd ----
    for (auto& h : hands) {
        if (h->mode == Mode::Hold)  prepare_hold_cmd(*h);
        if (h->mode == Mode::Drive) prepare_drive_cmd(*h);
    }

    // Drive plan banner.
    for (auto& h : hands) {
        if (h->mode != Mode::Drive) continue;
        std::cout << "   " << h->side << " DRIVE plan:\n";
        for (int i = 0; i < kMotorCount; ++i) {
            const bool act = h->is_active_joint[i];
            std::cout << "     [" << i << "] " << std::setw(9) << h->motor_name(i)
                      << "  startup=" << std::fixed << std::setprecision(3) << std::setw(7) << h->startup_q[i]
                      << "  open=" << std::setw(7) << h->open_q[i];
            if (h->is_active_joint[i]) {
                std::cout << "  closed=" << std::setw(7) << h->closed_q[i];
            } else {
                std::cout << "                  ";
            }
            std::cout << (act ? "  [active]" : "  [static]")
                      << (h->target_clamped[i] ? "  [CLAMPED]" : "")
                      << "\n";
        }
    }

    const bool any_publish =
        std::any_of(hands.begin(), hands.end(), [](const auto& h){ return h->mode != Mode::Read; });
    std::cout << ">>> " << (any_publish ? "Publishing" : "READ-ONLY (no cmd)") << ". Ctrl-C to quit.\n";

    // ---- CSV logging (parallel to collect's dex3_hand.csv for diffing) ----
    //
    // Column layout follows firmware wire order to match collect's csv and
    // Humanoid-Exo-Learning's Dex3 schema. Names are physical-joint-readable.
    //   L wire order: thumb_rotation, thumb_palm_bend, thumb_tip_bend,
    //                 middle_palm_bend, middle_tip_bend, index_palm_bend, index_tip_bend
    //   R wire order: thumb_rotation, thumb_palm_bend, thumb_tip_bend,
    //                 index_palm_bend, index_tip_bend, middle_palm_bend, middle_tip_bend
    constexpr const char* kJointAtSlotL[7] = {
        "thumb_rotation", "thumb_palm_bend", "thumb_tip_bend",
        "middle_palm_bend", "middle_tip_bend",
        "index_palm_bend", "index_tip_bend"};
    constexpr const char* kJointAtSlotR[7] = {
        "thumb_rotation", "thumb_palm_bend", "thumb_tip_bend",
        "index_palm_bend", "index_tip_bend",
        "middle_palm_bend", "middle_tip_bend"};

    constexpr int kPressEntries        = 9;
    constexpr int kPressValuesPerEntry = 12;

    auto build_header = [&]() {
        std::string s = "collection_id,host_timestamp";
        for (auto [side_label, names] : std::initializer_list<std::pair<const char*, const char* const*>>{
                {"L", kJointAtSlotL}, {"R", kJointAtSlotR}}) {
            for (const char* signal : {"cmd", "actual", "force"})
                for (int slot = 0; slot < kMotorCount; ++slot)
                    s += std::string(",") + side_label + "_" + signal + "_" + names[slot];
            for (int e = 0; e < kPressEntries; ++e)
                for (int c = 0; c < kPressValuesPerEntry; ++c)
                    s += std::string(",") + side_label + "_press_e"
                         + std::to_string(e) + "_c" + std::to_string(c);
        }
        return s;
    };

    const std::string csv_dir = repo_constants::DATA_DIR + "/dex3_test";
    std::filesystem::create_directories(csv_dir);
    CsvSaver test_csv(csv_dir + "/dex3_hand.csv", build_header());

    // ---- Main loop ----
    const auto publish_period_ms = std::max(1, 1000 / publish_rate_hz);
    const auto publish_period = std::chrono::milliseconds(publish_period_ms);
    const float dt_s = publish_period_ms / 1000.0f;
    const int   print_every = std::max(1, publish_rate_hz / std::max(1, print_rate_hz));
    int tick = 0;
    auto last_print = std::chrono::steady_clock::now();
    uint64_t last_msgs_L = 0, last_msgs_R = 0;
    while (true) {
        for (auto& h : hands) if (h->mode == Mode::Drive) step_drive_ramp(*h, dt_s);
        for (auto& h : hands) if (h->cmd_pub) h->cmd_pub->Write(h->cmd);

        // CSV row in firmware wire order (same shape as collect's dex3_hand.csv).
        {
            std::string line = "0," + std::to_string(Time::ts());
            for (auto& h : hands) {
                std::lock_guard<std::mutex> lock(h->state_mutex);
                for (int slot = 0; slot < kMotorCount; ++slot)
                    line += "," + std::to_string(h->current_cmd_q[slot]);
                for (int slot = 0; slot < kMotorCount; ++slot)
                    line += "," + std::to_string(h->state.motor_state()[slot].q());
                for (int slot = 0; slot < kMotorCount; ++slot)
                    line += "," + std::to_string(h->state.motor_state()[slot].tau_est());
                for (int e = 0; e < kPressEntries; ++e)
                    for (int c = 0; c < kPressValuesPerEntry; ++c)
                        line += "," + std::to_string(h->state.press_sensor_state()[e].pressure()[c]);
            }
            test_csv.write_line(line);
        }

        if (++tick % print_every == 0) {
            auto now = std::chrono::steady_clock::now();
            double dtp = std::chrono::duration<double>(now - last_print).count();
            last_print = now;
            uint64_t mL = hands[0]->state_msgs.load();
            uint64_t mR = hands[1]->state_msgs.load();
            double rateL = (mL - last_msgs_L) / dtp;
            double rateR = (mR - last_msgs_R) / dtp;
            last_msgs_L = mL; last_msgs_R = mR;

            for (auto& h : hands) {
                std::lock_guard<std::mutex> lock(h->state_mutex);

                // Always-on safety preview: per-finger curl -> intended palm-bend q.
                // Shown for both hands regardless of mode, so disabled hands act
                // as a dry-run before flipping enabled=true.
                const auto p = h->palm_idx();
                const float cur_t = h->curl_thumb.load();
                const float cur_i = h->curl_index.load();
                const float cur_m = h->curl_middle.load();
                const float q_t = h->target_q_for(p.thumb);
                const float q_i = h->target_q_for(p.index);
                const float q_m = h->target_q_for(p.middle);
                std::cout << std::fixed << std::setprecision(3)
                          << "[" << h->side << "] would-cmd  "
                          << "thumb: curl=" << cur_t << " q=" << std::showpos << q_t << std::noshowpos
                          << "   index: curl=" << cur_i << " q=" << std::showpos << q_i << std::noshowpos
                          << "   middle: curl=" << cur_m << " q=" << std::showpos << q_m << std::noshowpos
                          << "   [" << mode_str(h->mode)
                          << (h->cmd_pub ? "/PUB" : "/no-pub") << "]\n";

                std::cout << "[" << h->side << "/" << mode_str(h->mode) << "] meas = "
                          << fmt_q(h->state) << "\n";
                if (h->mode == Mode::Drive) {
                    std::ostringstream cmds;
                    cmds << std::fixed << std::setprecision(3);
                    float max_abs_err = 0.0f;
                    for (int i = 0; i < kMotorCount; ++i) {
                        cmds << (i ? "  " : "") << std::setw(6) << h->current_cmd_q[i];
                        float e = h->state.motor_state()[i].q() - h->current_cmd_q[i];
                        if (std::fabs(e) > max_abs_err) max_abs_err = std::fabs(e);
                    }
                    std::cout << "         cmd  = " << cmds.str()
                              << "   max|err|=" << std::fixed << std::setprecision(3) << max_abs_err << "\n";
                }
            }
            std::cout << "  state rate: L=" << std::fixed << std::setprecision(1) << rateL
                      << " Hz  R=" << rateR << " Hz   (msgs L=" << mL << " R=" << mR << ")\n";
            std::cout.flush();
        }
        std::this_thread::sleep_for(publish_period);
    }

    running.store(false);
    if (manus_thread.joinable()) manus_thread.join();
    return 0;
}
