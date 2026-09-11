#include "system/gearsonic_inference.hpp"

#include "common/config.hpp"
#include "common/console_tee.hpp"
#include "collector/motion_token_publisher.hpp"
#include "control/whole_body_controller.hpp"
#include "motion/input_handler.hpp"
#include "motion/nav_cmd_receiver.hpp"
#include "pico/pico_vr_reader.hpp"
#include "planner/planner_inference.hpp"
#include "teleop/teleop_tracker.hpp"
#include "unitree/hand_command_writer.hpp"
#include "unitree/hand_state_reader.hpp"
#include "unitree/unitree_command_writer.hpp"
#include "unitree/unitree_state_reader.hpp"
#include "vla/vla_token_receiver.hpp"

#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <thread>
#include <type_traits>

namespace kist {

static const std::string kConsoleLogPath = "logs/latest.log";

// config.yaml `hand:` section — every key optional, defaults in HandGuard::Params.
static HandGuard::Params parse_hand_guard(const YAML::Node& hand) {
    HandGuard::Params p;
    if (!hand) return p;
    auto get = [&](const char* key, auto& dst) {
        if (hand[key]) dst = hand[key].as<std::decay_t<decltype(dst)>>();
    };
    get("guard_enabled",  p.enabled);
    get("kp",             p.kp);
    get("kd",             p.kd);
    get("tau_max",        p.tau_max);
    get("stall_err_th",   p.stall_err_th);
    get("stall_vel_th",   p.stall_vel_th);
    get("stall_time_s",   p.stall_time_s);
    get("stall_offset",   p.stall_offset);
    get("kp_hold",        p.kp_hold);
    get("temp_max_c",     p.temp_max_c);
    get("state_stale_ms", p.state_stale_ms);
    return p;
}

GearsonicInference& GearsonicInference::instance() {
    static GearsonicInference inst;
    return inst;
}

bool GearsonicInference::start(const std::string& config_path) {
    // Mirror the console from the first line: one file per run, overwritten.
    ConsoleTee::instance().start(kConsoleLogPath);

    try {
        Config::instance().load(config_path);
    } catch (const std::exception& e) {
        std::cerr << "[GearsonicInference] config load failed: " << e.what() << "\n";
        return false;
    }
    auto root = Config::instance().root();

    // ── input stack (nothing moves yet) ─────────────────────────
    // VR is mandatory: the joystick drives the planner and the
    // controller (A+B+X+Y held 1s) is the operator e-stop.
    if (!PicoVRReader::instance().start())
        return false;
    vr_started_ = true;
    InputHandler::instance().start();
    TeleopTracker::instance().start();

    if (!UnitreeStateReader::instance().start(
            root["unitree"]["domain_id"].as<int>(),
            root["unitree"]["network_interface"].as<std::string>())) {
        stop();
        return false;
    }
    robot_started_ = true;

    // Dex3-1 feedback rides the same DDS factory. Pure intake: nothing
    // waits on it, so a robot without hands still comes up — the hand
    // writer just never sees state and stays open-loop.
    if (!HandStateReader::instance().start()) {
        stop();
        return false;
    }
    hand_state_started_ = true;

    // VLA latent-action Rx rides the DDS factory UnitreeStateReader just
    // initialized. Pure intake — nothing moves until tokens arrive AND the
    // controller is in CONTROL (first-come arbitration, control_arbiter.hpp).
    if (!VlaTokenReceiver::instance().start()) {
        stop();
        return false;
    }
    vla_started_ = true;

    // NAV command Rx: base-frame velocity (vx,vy,vyaw) from the navigation planner
    // (kist-navigation-planner) -> InputHandler::nav_buf, on the same DDS factory.
    // Pure intake; the stick-vs-nav arbitration lives in input_handler.
    if (!NavCmdReceiver::instance().start()) {
        stop();
        return false;
    }
    nav_rx_started_ = true;

    std::cout << "[GearsonicInference] waiting for robot state...\n";
    while (!UnitreeStateReader::instance().unitree_state_buf.GetData()) {
        if (quit_) {
            stop();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // ── planner ─────────────────────────────────────────────────
    auto& planner = PlannerInference::instance();
    if (!planner.start(root["planner"]["model_path"].as<std::string>())) {
        stop();
        return false;
    }
    planner_started_ = true;

    auto& control = WholeBodyController::instance();
    planner.set_playback_provider([&control](MotionSequence50Hz& m, int& c) {
        return control.playback_snapshot(m, c);
    });

    // ── actuation: THE ROBOT MOVES FROM HERE ────────────────────
    if (!UnitreeCommandWriter::instance().start(&control.motor_command_buf)) {
        stop();
        return false;
    }
    writer_started_ = true;

    // Dex3-1 hands ride the same DDS factory; the trigger stream drives them
    // orthogonally to the WBC policy (policy outputs 29 arm/leg joints only).
    if (!HandCommandWriter::instance().start(parse_hand_guard(root["hand"]))) {
        stop();
        return false;
    }
    hand_writer_started_ = true;

    // Motion-token record stream (rt/kist/motion_token) for the data
    // collector — pure observer of the decoder-input token; nothing in the
    // control path waits on it.
    if (!MotionTokenPublisher::instance().start(&control.motion_token_buf)) {
        stop();
        return false;
    }
    token_pub_started_ = true;

    if (!control.start(root["control"]["encoder_path"].as<std::string>(),
                       root["control"]["decoder_path"].as<std::string>(),
                       /*auto_start_control=*/true)) {
        stop();
        return false;
    }
    control_started_ = true;

    std::cout << "[GearsonicInference] started — INIT ramp, then policy control\n";
    return true;
}

void GearsonicInference::stop() {
    // Controller first. It flushes damping into the command buffer,
    // then the still-running writer publishes it before its own stop()
    // sends the final damping burst.
    if (control_started_) {
        WholeBodyController::instance().stop();
        control_started_ = false;
    }
    if (token_pub_started_) {
        MotionTokenPublisher::instance().stop();
        token_pub_started_ = false;
    }
    if (hand_writer_started_) {
        HandCommandWriter::instance().stop();
        hand_writer_started_ = false;
    }
    if (writer_started_) {
        UnitreeCommandWriter::instance().stop();
        writer_started_ = false;
    }
    if (hand_state_started_) {
        HandStateReader::instance().stop();
        hand_state_started_ = false;
    }
    if (planner_started_) {
        PlannerInference::instance().stop();
        planner_started_ = false;
    }
    if (nav_rx_started_) {
        NavCmdReceiver::instance().stop();
        nav_rx_started_ = false;
    }
    if (vla_started_) {
        VlaTokenReceiver::instance().stop();
        vla_started_ = false;
    }
    if (vr_started_) {
        TeleopTracker::instance().stop();
        InputHandler::instance().stop();
    }
    if (robot_started_) {
        UnitreeStateReader::instance().stop();
        robot_started_ = false;
    }
    if (vr_started_) {
        PicoVRReader::instance().stop();
        vr_started_ = false;
    }
    ConsoleTee::instance().stop();
}

// ── signals ───────────────────────────────────────────────────────────────────

static void on_signal(int) {
    GearsonicInference::instance().request_quit();
}

void GearsonicInference::install_signal_handlers() {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
}

} // namespace kist
