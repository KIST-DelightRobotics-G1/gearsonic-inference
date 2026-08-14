#pragma once

#include <atomic>
#include <string>

namespace kist {

// System-level facade — the single owner of module wiring and startup
// order. Consumers (the gearsonic binary, an embedding repo) call this
// instead of assembling the modules themselves.
//
//   GearsonicInference::instance().start("config/config.yaml");
//     -> PicoVRReader -> InputHandler -> TeleopTracker
//        -> UnitreeStateReader (waits for robot state) -> VlaTokenReceiver
//        -> PlannerInference (+ playback provider wiring)
//        -> UnitreeCommandWriter -> HandCommandWriter
//        -> MotionTokenPublisher -> WholeBodyController
//        -> 3s INIT ramp -> policy control, no further calls needed.
//
// *** Calling start() means the robot WILL MOVE. *** Gating (operator
// confirmation, UI) is the caller's responsibility.
//
// Safety contract:
//   1. stop() always leaves damping as the last published command.
//   2. Getting stop() called on every exit path is the caller's job —
//      either wire your signal handler to it or call
//      install_signal_handlers() and poll quit_requested().
//   3. Process death (SIGKILL, crash) cannot run any code: the physical
//      remote's damping is the last line of defense. Keep it in reach.
//
// Data in/out stays on the module singletons (this class is lifecycle
// only): InputHandler::nav_buf, mode(), estop(),
// WholeBodyController::state(), encoder_mode(), last_timing(), ...
class GearsonicInference {
public:
    static GearsonicInference& instance();

    // Full-chain startup. Blocks until robot state is flowing, then the
    // INIT ramp and the policy handover proceed on their own. Returns
    // false (with everything already-started rolled back) on any failure.
    bool start(const std::string& config_path);

    // Shutdown, controller first; publishes damping before the writer
    // stops. Idempotent — safe to call after a failed start() or twice.
    void stop();

    // Optional: SIGINT/SIGTERM -> quit_requested() becomes true. The
    // caller's main loop polls it and calls stop(). Not installed by
    // default — embedding processes (e.g. ROS2) manage their own signals
    // and can call request_quit() from them instead.
    void install_signal_handlers();
    void request_quit() { quit_ = true; }
    bool quit_requested() const { return quit_; }

private:
    GearsonicInference() = default;

    // Which stages came up, so stop()/rollback only touches what runs.
    bool vr_started_{false};
    bool robot_started_{false};
    bool vla_started_{false};
    bool planner_started_{false};
    bool writer_started_{false};
    bool hand_writer_started_{false};
    bool token_pub_started_{false};
    bool control_started_{false};

    std::atomic<bool> quit_{false};
};

} // namespace kist
