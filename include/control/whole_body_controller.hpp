#pragma once

#include "common/data_buffer.hpp"
#include "control/motion_token.hpp"
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

    // Recording tap: a copy of every token the decoder consumed, for the
    // MotionTokenPublisher (pure observer — the control path only does a
    // SetData here, exactly like motor_command_buf above).
    DataBuffer<MotionTokenSample> motion_token_buf;

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

    // ── arbitration (see control_arbiter.hpp) ──────────────────
    // This loop is the ControlArbiter stage's single writer:
    // update_arbiter() runs step_mode() each CONTROL tick and applies the
    // EXTERNAL side effects of a transition (calibration reset, playback
    // reset, locomotion disarm, planner reseed ticket, logs) — the token
    // selection and blends live inside the stage. tick_vla feeds it the
    // stream's token; tick_recovery lets it hold the standing blend while
    // this loop reseeds the planner so kNormal resumes from a fresh origin
    // (never the frozen pre-VLA timeline).
    void update_arbiter();
    void tick_vla(std::chrono::steady_clock::time_point t0);
    void tick_recovery(std::chrono::steady_clock::time_point t0);
    void decode_and_publish(const TokenEncoder::Token& token,
                            std::chrono::steady_clock::time_point t0);

    // Drop a copy of the consumed token into motion_token_buf (called
    // after the tick's MotorCommand is published, never before).
    void record_token(const TokenEncoder::Token& token, bool encoder_ran);
    uint64_t token_seq_{0};

    bool     reinit_ticket_issued_{false};    // planner reseed asked this recovery
    uint64_t reinit_ticket_{0};               // planner ack target
    bool     last_vr3_present_{false};        // teleop engage/disengage edge

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
