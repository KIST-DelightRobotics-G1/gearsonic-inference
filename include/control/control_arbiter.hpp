#pragma once

#include "vla/vla_initial_pose.hpp"
#include "vla/vla_latent_action.hpp"

#include <array>
#include <atomic>
#include <cstddef>

namespace kist {

// Control-source arbitration stage, sitting on the token path between the
// TokenEncoder and the PolicyDecoder: first-come mutual exclusion between
// the external VLA token stream and VR teleop, with every exit routed
// through the origin (planner idle) — never a direct handover.
//
//   kNormal ──B calib──> kTeleop ──B off / VR loss──> kNormal
//   kNormal ──fresh token──> kVla ──stream lost──> kRecovering ──> kNormal
//
// Rules (deliberately no priority, no preemption):
//   - Entries happen only from kNormal; whoever claims first keeps the
//     robot until it lets go. The other side's inputs are ignored while
//     owned (teleop's B gesture during kVla/kRecovering, VLA tokens during
//     kTeleop).
//   - kVla exit (stream stale) runs the recovery sequence: blend to the
//     safe standing token, hold balancing, reseed the planner from the
//     measured standing pose, then hand the origin back. A resumed stream
//     re-claims from kNormal — no restart needed.
//   - At the origin a live token stream claims automatically each tick,
//     so while VLA keeps publishing, teleop cannot engage (B needs a 1s
//     hold); stop the publisher first to hand the robot to an operator.
//   - The VR e-stop (A+B+X+Y held 1s) is NOT part of this arbitration: it
//     overrides everything, always, in both control loops.
//
// Stage anatomy (WholeBodyController's 50Hz loop is the single writer;
// HandCommandWriter / TeleopTracker / InputHandler only read mode()):
//   step_mode()     — decide + arm the token blends on transitions
//   select_token()  — exactly one token out per tick, whatever the mode:
//                     the encoder's (kNormal/kTeleop, crossfaded out of a
//                     recovery), the stream's (kVla), or the standing
//                     blend (kRecovering)
// External side effects of a transition (playback reset, the planner
// reseed call, locomotion disarm, logging) stay in the controller — this
// stage never touches another module.
class ControlArbiter {
public:
    enum class Mode { kNormal, kTeleop, kVla, kRecovering };

    // 64-dim SONIC latent — same type as TokenEncoder::Token and
    // VlaLatentAction::token (kept as a plain array here so the readers of
    // mode() don't inherit the encoder's TensorRT includes).
    using Token = std::array<float, 64>;

    // The singleton is the production instance; the constructor stays
    // public so probes can run a private state machine (test_arbiter_probe).
    ControlArbiter() = default;
    static ControlArbiter& instance() {
        static ControlArbiter inst;
        return inst;
    }

    Mode mode() const { return mode_.load(); }

    // Pure transition function — all inputs injected, no side effects.
    //   vla_fresh     : token age <= kVlaTokenFreshMs (claim threshold)
    //   vla_alive     : token age <= kVlaTokenHoldMs  (session tolerance)
    //   calibrated    : TeleopTracker::calibrated()
    //   recovery_done : the planner reseed this recovery asked for was
    //                   acknowledged
    static Mode decide(Mode current, bool vla_fresh, bool vla_alive,
                       bool calibrated, bool recovery_done) {
        switch (current) {
            case Mode::kNormal:
                if (vla_fresh)  return Mode::kVla;
                if (calibrated) return Mode::kTeleop;
                return Mode::kNormal;
            case Mode::kTeleop:
                // VLA ignored while an operator holds the robot.
                return calibrated ? Mode::kTeleop : Mode::kNormal;
            case Mode::kVla:
                // B ignored while VLA holds (TeleopTracker gates on mode).
                return vla_alive ? Mode::kVla : Mode::kRecovering;
            case Mode::kRecovering:
                return recovery_done ? Mode::kNormal : Mode::kRecovering;
        }
        return current;  // unreachable
    }

    // ── stage face (the controller's 50Hz tick only) ────────────

    // decide() + blend arming. Returns the new mode; the caller compares
    // against the previous one for its own transition side effects.
    Mode step_mode(bool vla_fresh, bool vla_alive, bool calibrated,
                   bool recovery_done) {
        Mode cur  = mode_.load();
        Mode next = decide(cur, vla_fresh, vla_alive, calibrated, recovery_done);
        if (next != cur) {
            if (next == Mode::kRecovering)
                recovery_blend_tick_ = 0;
            if (cur == Mode::kRecovering && next == Mode::kNormal)
                handoff_blend_ticks_ = kHandoffBlendTicks;
            mode_.store(next);
        }
        return next;
    }

    // True once the standing blend has landed — the recovery may reseed
    // the planner from here (the caller issues the ticket, once).
    bool recovery_blend_done() const {
        return recovery_blend_tick_ >= kRecoveryBlendTicks;
    }

    // Exactly one token out per tick. Pass the token the current mode
    // produces (encoder's in kNormal/kTeleop, the stream's in kVla,
    // nothing in kRecovering); the blends are applied here.
    Token select_token(const Token* encoder_token, const Token* vla_token) {
        switch (mode_.load()) {
            case Mode::kVla:
                held_token_ = *vla_token;
                return held_token_;
            case Mode::kRecovering: {
                // Blend the last commanded token to the verified safe
                // standing token and keep balancing there; track the hold
                // so the handoff out of recovery starts from it.
                if (recovery_blend_tick_ < kRecoveryBlendTicks)
                    ++recovery_blend_tick_;
                float a = static_cast<float>(recovery_blend_tick_) / kRecoveryBlendTicks;
                for (size_t i = 0; i < held_token_.size(); ++i)
                    held_token_[i] = (1.0f - a) * held_token_[i]
                                     + a * kVlaSafeStandingToken[i];
                return held_token_;
            }
            default: {
                // Fresh out of recovery: crossfade from the standing hold
                // so the switch back to the encoder is not a step input.
                Token out = *encoder_token;
                if (handoff_blend_ticks_ > 0) {
                    float w = 1.0f - static_cast<float>(handoff_blend_ticks_)
                                         / kHandoffBlendTicks;
                    for (size_t i = 0; i < out.size(); ++i)
                        out[i] = (1.0f - w) * held_token_[i] + w * out[i];
                    --handoff_blend_ticks_;
                }
                return out;
            }
        }
    }

    static constexpr int kRecoveryBlendTicks = kVlaLossBlendTicks;  // 1s at 50Hz
    static constexpr int kHandoffBlendTicks  = 25;                  // 0.5s at 50Hz

private:
    std::atomic<Mode> mode_{Mode::kNormal};

    // Blend state (single-writer: the controller's tick)
    Token held_token_{};          // last commanded token (VLA / standing hold)
    int   recovery_blend_tick_{0};
    int   handoff_blend_ticks_{0};
};

} // namespace kist
