#pragma once

#include "common/data_buffer.hpp"
#include "control/motor_command.hpp"
#include "control/policy_decoder.hpp"
#include "control/state_logger.hpp"
#include "control/token_encoder.hpp"
#include "planner/motion_sequence_50hz.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace kist {

// 50Hz whole-body controller, ported from gear_sonic's Control() thread:
//   INIT (3s ramp to default pose) -> WAIT_FOR_CONTROL -> CONTROL
//
// The orchestrator: owns the state machine, safety gates (e-stop,
// LowState), robot-state history, and playback of the planner motion
// (blending + cursor). The per-tick computation lives in the two stages
// it drives: TokenEncoder (motion -> token) and PolicyDecoder
// (token + history -> MotorCommand).
//
// It never touches DDS output itself — the 500Hz command writer consumes
// motor_command_buf; without the writer running this is inherently dry.
class WholeBodyController {
public:
    enum class State { INIT, WAIT_FOR_CONTROL, CONTROL, ESTOP };

    static WholeBodyController& instance();
    bool start(const std::string& encoder_path, const std::string& decoder_path,
               bool auto_start_control = false);
    void stop();

    // Operator "start" (WAIT_FOR_CONTROL -> CONTROL transition)
    void start_control() { operator_start_ = true; }

    State state() const { return state_; }

    // Encoder mode of the last CONTROL tick (g1=0, teleop=1, -1 before first)
    int encoder_mode() const { return encoder_.mode(); }

    // ── outputs ─────────────────────────────────────────────────
    DataBuffer<MotorCommand> motor_command_buf;

    // Playback snapshot for the planner's context (blended motion + cursor).
    // Returns false until the first planner motion has been adopted.
    bool playback_snapshot(MotionSequence50Hz& motion, int& cursor) const;

    // ── timing (µs, last CONTROL tick) ──────────────────────────
    struct Timing { long encoder{0}, decoder{0}, total{0}; };
    Timing last_timing() const { std::lock_guard<std::mutex> l(timing_mutex_); return timing_; }

private:
    WholeBodyController() = default;

    void loop();
    bool check_safety();
    void publish_damping();
    bool gather_robot_state();
    void tick_init();
    void tick_control();
    void advance_playback();

    // ── VLA external-token mode ─────────────────────────────────
    // tick_vla_control returns true when it owned this tick (external
    // token consumed, or holding the safe standing blend after the
    // stream was lost).
    bool tick_vla_control(std::chrono::steady_clock::time_point t0);
    bool vla_active_{false};
    bool vla_stream_lost_{false};
    bool vla_denied_logged_{false};
    int  vla_loss_blend_tick_{0};
    TokenEncoder::Token last_vla_token_{};

    TokenEncoder  encoder_;
    PolicyDecoder decoder_;
    StateLogger   logger_;

    // Playback state (blended planner motion + cursor)
    mutable std::mutex playback_mutex_;
    MotionSequence50Hz playback_motion_;
    int                cursor_{0};
    bool               playing_{false};
    std::chrono::steady_clock::time_point last_plan_stamp_{};

    // INIT ramp
    int init_ticks_{0};

    std::atomic<State> state_{State::INIT};
    std::atomic<bool>  operator_start_{false};
    bool               auto_start_{false};

    mutable std::mutex timing_mutex_;
    Timing             timing_;

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};

    static constexpr double kControlDt   = 0.02;   // 50Hz
    static constexpr int    kInitTicks   = 150;    // 3s ramp
    static constexpr int    kBlendFrames = 8;      // planner crossfade width
};

} // namespace kist
