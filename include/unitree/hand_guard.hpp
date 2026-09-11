#pragma once

#include "unitree/hand_command.hpp"
#include "unitree/hand_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace kist {

// Per-hand protection stage between target selection and publish.
//
// The hand firmware runs a fixed PD:
//     tau = kp*(q_cmd - q_meas) + kd*(dq_cmd - dq_meas)
// We only choose q_cmd/kp/kd each tick, so the stall torque is bounded by
// choosing q_cmd within e_max = tau_max/kp of the measured position:
//     q_cmd = q_meas + clamp(q_des - q_meas, -e_max, +e_max)
// which makes the firmware compute at most kp*e_max when the finger is
// blocked (dq_meas = 0). On top of that a stall detector — large error
// AND no motion for stall_time_s — latches the motor onto
//     q_cmd = q_meas + stall_offset*sign(q_des - q_meas),  kp = kp_hold
// so a finger pressed against an obstacle keeps only a light contact
// torque instead of heating at the clamp limit. The latch releases when
// the desired target comes back within stall_err_th of the finger.
//
// Without fresh measurements (no hands, stale stream) the stage passes
// the desired command through unchanged, i.e. the previous open-loop
// behaviour with the writer's low fallback gains.
class HandGuard {
public:
    struct Params {
        bool   enabled{true};
        // Tracking gains used while measurements are fresh.
        float  kp{1.5f};
        float  kd{0.1f};
        // Stall torque bound: e_max = tau_max / kp.
        float  tau_max{0.6f};
        // Stall detector.
        float  stall_err_th{0.15f};   // rad — |q_des - q_meas| above this ...
        float  stall_vel_th{0.05f};   // rad/s — ... while |dq_meas| below this ...
        float  stall_time_s{0.3f};    // ... for this long -> latched
        float  stall_offset{0.25f};   // rad kept toward the target while latched
        float  kp_hold{1.5f};         // kp while latched
        int    temp_max_c{0};         // casing deg C that latches immediately (0 = off)
        double state_stale_ms{100.0}; // measurements older than this count as absent
    };

    HandGuard() = default;
    HandGuard(bool is_left, const Params& p) : is_left_(is_left), p_(p) {}

    void configure(bool is_left, const Params& p) {
        is_left_ = is_left; p_ = p; reset();
    }

    const Params& params() const { return p_; }

    // meas == nullptr -> no fresh measurement: pass through.
    HandCommand apply(const HandCommand& desired, const HandState* meas, double dt_s) {
        if (!p_.enabled || !desired.enabled || meas == nullptr) {
            reset();
            return desired;
        }
        const auto& lo = is_left_ ? kDex3LeftMin : kDex3RightMin;
        const auto& hi = is_left_ ? kDex3LeftMax : kDex3RightMax;
        const float e_max = p_.tau_max / std::max(p_.kp, 1e-3f);

        HandCommand out = desired;
        for (int i = 0; i < kHandMotors; ++i) {
            const float q_meas  = meas->motors[i].q;
            const float dq_meas = meas->motors[i].dq;
            const float e       = desired.q[i] - q_meas;

            // ── stall detector ──
            bool cond = std::fabs(e) > p_.stall_err_th && std::fabs(dq_meas) < p_.stall_vel_th;
            bool hot  = p_.temp_max_c > 0 && meas->motors[i].temp_casing >= p_.temp_max_c;
            stall_s_[i] = cond ? stall_s_[i] + static_cast<float>(dt_s) : 0.0f;
            bool latched = latched_[i];
            if (!latched && (hot || stall_s_[i] >= p_.stall_time_s)) latched = true;
            if (latched && !hot && std::fabs(e) <= p_.stall_err_th)  latched = false;
            latched_[i] = latched;

            // ── q_cmd within reach of q_meas ──
            float q_cmd;
            if (latched) {
                q_cmd    = q_meas + p_.stall_offset * (e > 0 ? 1.0f : (e < 0 ? -1.0f : 0.0f));
                out.kp[i] = p_.kp_hold;
            } else {
                q_cmd    = q_meas + std::clamp(e, -e_max, e_max);
                out.kp[i] = p_.kp;
            }
            out.q[i]  = std::clamp(q_cmd, lo[i], hi[i]);
            out.kd[i] = p_.kd;
        }
        return out;
    }

    void reset() {
        stall_s_.fill(0.0f);
        latched_.fill(false);
    }

    bool latched(int motor) const { return latched_[motor]; }

private:
    bool        is_left_{true};
    Params      p_{};
    std::array<float, kHandMotors> stall_s_{};
    std::array<bool,  kHandMotors> latched_{};
};

} // namespace kist
