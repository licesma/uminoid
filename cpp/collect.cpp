#include "camera/camera_recorder.hpp"
#include "camera/preview_server.hpp"
#include "camera/usb_camera_recorder.hpp"
#include "camera/zmq_camera_recorder.hpp"
#include "collect_ui.hpp"
#include "hand_retarget/dex3/dex3_retargeter.hpp"
#include "hand_retarget/hand_retargeter.hpp"
#include "hand_retarget/inspire/g1_inspire_retargeter.hpp"
#include "hand_retarget/inspire/usb_inspire_retargeter.hpp"
#include "upper_body_reader/exo_upper_body_reader.hpp"
#include "upper_body_reader/g1_upper_body_reader.hpp"
#include "utils/recording_label.hpp"
#include "utils/repo_constants.hpp"

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace config {
    // Runtime configuration lives in <repo>/config, a sibling of cpp/.
    static const std::filesystem::path CONFIG_DIR =
        (std::filesystem::path(__FILE__).parent_path() / ".." / "config").lexically_normal();
    static const std::string PATH = (CONFIG_DIR / "collect.yaml").string();
    static const YAML::Node yaml = YAML::LoadFile(PATH);

    // Per-machine device identifiers (serial ports, hand ids) live separately.
    static const std::string DEVICES_PATH = (CONFIG_DIR / "devices.yaml").string();
    static const YAML::Node devices = YAML::LoadFile(DEVICES_PATH);

    inline const bool upper_body_enabled = yaml["upper_body"]["enabled"].as<bool>();
    inline const bool hand_enabled       = yaml["hand"]["enabled"].as<bool>();
    inline const bool camera_enabled     = yaml["camera"]["enabled"].as<bool>();
    inline const std::string upper_body_source = yaml["upper_body"]["source"] ? yaml["upper_body"]["source"].as<std::string>() : std::string("exo");
    inline const std::string camera_source = yaml["camera"]["source"] ? yaml["camera"]["source"].as<std::string>() : std::string("usb");

    inline const std::string left_arm   = devices["arms"]["left"].as<std::string>();
    inline const std::string right_arm  = devices["arms"]["right"].as<std::string>();
    inline const bool        upper_body_left_enabled  =
        yaml["upper_body"]["left_enabled"]
            ? yaml["upper_body"]["left_enabled"].as<bool>()
            : true;
    inline const bool        upper_body_right_enabled =
        yaml["upper_body"]["right_enabled"]
            ? yaml["upper_body"]["right_enabled"].as<bool>()
            : true;
    inline const int         baudrate   = yaml["upper_body"]["baudrate"].as<int>();
    inline const bool is_simulation = yaml["is_simulation"].as<bool>();

    // The DDS network interface is fully determined by simulation mode: the
    // simulator publishes on loopback, the real robot on its wired NIC. Shared
    // by the G1 body (upper_body source "g1") and the hands (DDS sources), so
    // it is derived here rather than duplicated in the config.
    inline constexpr const char* SIM_NETWORK_INTERFACE  = "lo";
    inline constexpr const char* REAL_NETWORK_INTERFACE = "enx00e04c6803fc";
    inline const std::string network_interface =
        is_simulation ? SIM_NETWORK_INTERFACE : REAL_NETWORK_INTERFACE;

    inline const DynamicsModel dynamics_model = parse_dynamics_model(
        yaml["dynamics"] && yaml["dynamics"]["model"]
            ? yaml["dynamics"]["model"].as<std::string>()
            : std::string("baseline"));

    inline const std::string hand_source =
        yaml["hand"]["source"] ? yaml["hand"]["source"].as<std::string>() : std::string("inspire_usb");
    inline const bool    hand_left_enabled  = yaml["hand"]["left"]["enabled"].as<bool>();
    inline const bool    hand_right_enabled = yaml["hand"]["right"]["enabled"].as<bool>();
    inline const uint8_t hand_left_id       = devices["hands"]["left_id"]
        ? devices["hands"]["left_id"].as<int>()  : 0;
    inline const uint8_t hand_right_id      = devices["hands"]["right_id"]
        ? devices["hands"]["right_id"].as<int>() : 0;

    // Preview is optional; an empty/missing bind_host disables it entirely.
    inline PreviewServer::Config load_preview_cfg() {
        PreviewServer::Config c;
        if (!yaml["preview"]) return c;
        const auto& p = yaml["preview"];
        if (p["bind_host"])    c.bind_host    = p["bind_host"].as<std::string>();
        if (p["port"])         c.port         = p["port"].as<int>();
        if (p["width"])        c.out_width    = p["width"].as<int>();
        if (p["height"])       c.out_height   = p["height"].as<int>();
        if (p["jpeg_quality"]) c.jpeg_quality = p["jpeg_quality"].as<int>();
        if (p["max_fps"])      c.max_fps      = p["max_fps"].as<int>();
        return c;
    }
    inline const PreviewServer::Config preview_cfg = load_preview_cfg();
    inline const bool preview_enabled = camera_enabled && !preview_cfg.bind_host.empty();
}

int main() {
    const std::string recording_label = generate_recording_label();

    std::atomic<bool> running{true};
    std::atomic<bool> _paused{true};
    std::atomic<int>  _collection_id{1};
    std::atomic<bool> error_reported{false};
    std::string       error_msg;
    std::mutex        error_msg_mutex;

    auto stop          = [&running]        { return !running.load(); };
    auto paused        = [&_paused]        { return _paused.load(); };
    auto collection_id = [&_collection_id] { return _collection_id.load(); };
    auto raise_error   = [&](const std::string& msg) {
        if (error_reported.exchange(true)) return;
        {
            std::lock_guard<std::mutex> lock(error_msg_mutex);
            error_msg = msg;
        }
        ui::cancel_current(_collection_id.load());
        running.store(false);
    };

    std::filesystem::create_directories(repo_constants::DATA_DIR + "/" + recording_label);

    std::unique_ptr<PreviewServer> preview;
    if (config::preview_enabled) {
        preview = std::make_unique<PreviewServer>(config::preview_cfg, raise_error);
        preview->start();
        std::cout << "  preview: http://" << config::preview_cfg.bind_host
                  << ":" << config::preview_cfg.port << "/\n";
    }

    std::unique_ptr<UpperBodyReader> upper_body;
    std::unique_ptr<HandRetargeter>  hand;
    std::unique_ptr<CameraRecorder>  camera;

    if (config::upper_body_enabled) {
        if (config::upper_body_source == "g1") {
            G1UpperBodyReaderConfig g1_config{};
            g1_config.controller.network_interface = config::network_interface;
            g1_config.controller.is_simulation = config::is_simulation;
            g1_config.controller.recording_label = recording_label;
            g1_config.controller.left_enabled = config::upper_body_left_enabled;
            g1_config.controller.right_enabled = config::upper_body_right_enabled;
            g1_config.controller.dynamics_model = config::dynamics_model;
            g1_config.left_device = config::left_arm;
            g1_config.right_device = config::right_arm;
            g1_config.baudrate = config::baudrate;
            upper_body = std::make_unique<G1UpperBodyReader>(g1_config, raise_error);
        } else {
            upper_body = std::make_unique<ExoUpperBodyReader>(
                config::left_arm, config::right_arm, config::baudrate,
                recording_label,
                config::upper_body_left_enabled,
                config::upper_body_right_enabled,
                raise_error);
        }
    }
    if (config::hand_enabled) {
        if (config::hand_source == "inspire_g1") {
            hand = std::make_unique<G1InspireRetargeter>(
                config::hand_left_enabled,
                config::hand_right_enabled,
                config::network_interface,
                recording_label, raise_error);
        } else if (config::hand_source == "dex3") {
            hand = std::make_unique<Dex3Retargeter>(
                config::hand_left_enabled,
                config::hand_right_enabled,
                config::network_interface,
                recording_label, raise_error);
        } else if (config::hand_source == "inspire_usb") {
            hand = std::make_unique<UsbInspireRetargeter>(
                config::hand_left_enabled,  config::hand_left_id,
                config::hand_right_enabled, config::hand_right_id,
                recording_label, raise_error);
        } else {
            raise_error("hand.source must be one of: inspire_usb, inspire_g1, dex3 (got '"
                        + config::hand_source + "')");
        }
    }
    if (config::camera_enabled) {
        if (config::camera_source == "zmq") {
            camera = std::make_unique<ZmqCameraRecorder>(recording_label, raise_error, preview.get());
        } else {
            camera = std::make_unique<UsbCameraRecorder>(recording_label, raise_error, preview.get());
        }
    }

    std::cout << "  " << recording_label
              << "   space → next   x → cancel   q → stop\n";

    ui::add_next(_collection_id.load());

    std::thread camera_thread, upper_body_thread, hand_thread;
    if (camera)     camera_thread     = std::thread([&] { camera->collect_loop(collection_id, stop, paused); });
    if (upper_body) upper_body_thread = std::thread([&] { upper_body->collect_loop(collection_id, stop, paused); });
    if (hand)       hand_thread       = std::thread([&] { hand->retarget_loop(stop, collection_id, paused); });

    std::thread display_thread([&] {
        while (running.load()) {
            ui::redraw();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        ui::redraw();
    });

    {
        RawMode raw;
        while (running.load()) {
            char key = 0;
            if (read(STDIN_FILENO, &key, 1) <= 0)
                continue;

            if (key == 'q' || key == 'Q' || key == 3 /* Ctrl+C */) {
                ui::complete_current(_collection_id.load());
                running.store(false);
            } else if (key == ' ') {
                if (!_paused.load()) {
                    // Collecting → pause: complete current, queue next
                    ui::complete_current(_collection_id.load());
                    _collection_id.fetch_add(1);
                    ui::add_next(_collection_id.load());
                    _paused.store(true);
                } else {
                    // Paused → resume: start the queued Next collection
                    if (ui::start_next_collection())
                        _paused.store(false);
                }
            } else if (key == 'x' || key == 'X') {
                if (!_paused.load()) {
                    // Collecting → cancel current, queue next
                    ui::cancel_current(_collection_id.load());
                    _collection_id.fetch_add(1);
                    ui::add_next(_collection_id.load());
                    _paused.store(true);
                }
            } else if (upper_body) {
                upper_body->handle_key(key);
            }
        }
    }

    ui::save_status_csv(repo_constants::DATA_DIR + "/" + recording_label + "/collection_status.csv");

    display_thread.join();
    if (camera_thread.joinable())  camera_thread.join();
    if (upper_body_thread.joinable()) upper_body_thread.join();
    if (hand_thread.joinable()) hand_thread.join();

    if (preview) preview->stop();

    if (error_reported.load()) {
        std::lock_guard<std::mutex> lock(error_msg_mutex);
        std::cerr << "\n[FATAL] " << error_msg << "\n";
    }

    return 0;
}
