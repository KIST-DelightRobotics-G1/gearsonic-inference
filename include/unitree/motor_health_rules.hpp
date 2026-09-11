#pragma once

#include "control/motor_command.hpp"
#include "unitree/g1_motor_names.hpp"
#include "unitree/hand_state.hpp"
#include "unitree/unitree_state.hpp"

#include <array>
#include <cmath>
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

namespace kist {

// Fault detection over the per-motor fields the SDK already reports plus
// what we command. Pure: step() takes the latest snapshots and returns the
// log lines to emit this tick (edge-triggered — a condition prints when it
// appears and when it clears, not every tick). Log-only by decision: what
// the robot should DO about a fault is settled once real fault logs show
// which conditions occur and how the SDK reports them.
//
// Conditions, per body motor:
//  - FAULT        : SDK motorstate word != 0 (the flags the Unitree app
//                   shows, e.g. under-voltage). Raw hex is logged.
//  - TEMPERATURE  : two sensors per motor, judged separately — casing
//                   (temperature[0]) and winding (temperature[1]). The SDK
//                   cuts the robot out at casing 85C / winding 120C
//                   (unitree/robot/g1/common/terminations.hpp). WARNING is
//                   raised at 85% of that (72 / 102) so a person sees the
//                   trend, EMERGENCY at 95% (80 / 114) so it is declared
//                   before the cut-out, not at it. Clears 5C below warn.
//  - NOT RESPONDING: the motor is not producing the torque its command
//                   asks for. The policy uses the firmware PD as a torque
//                   generator — a standing robot holds ankle/hip targets
//                   0.3..0.8 rad away from the measured pose on purpose —
//                   so position error means nothing here. What a live
//                   motor always does is report tau_est close to
//                   tau_expected = kp*(q_cmd - q) - kd*dq. Flag when
//                   |tau_expected| > tau_min and |tau_est| < ratio *
//                   |tau_expected| for unresp_time_s. A blocked joint still
//                   produces torque and is not flagged; a dead driver is.
//  - STATE FROZEN : q_meas bit-identical for frozen_time_s while the
//                   command moved > frozen_cmd_move — an encoder/driver
//                   that stopped reporting (the observation is a lie).
// Hands: FAULT (error word) and TEMPERATURE only.
class MotorHealthRules {
public:
    struct Params {
        bool   enabled{true};
        int    casing_warn_c{72};       // 85% of the SDK cut-out 85 (motor_casing_overheat)
        int    casing_critical_c{80};   // 95% of the SDK cut-out 85
        int    winding_warn_c{102};     // 85% of the SDK cut-out 120 (motor_winding_overheat)
        int    winding_critical_c{114}; // 95% of the SDK cut-out 120
        float  unresp_tau_min{5.0f};      // Nm the command must ask for
        float  unresp_tau_ratio{0.3f};    // measured/expected below this = not producing
        float  unresp_time_s{1.0f};
        float  frozen_time_s{1.0f};
        float  frozen_cmd_move{0.02f};    // rad the command must travel
        float  summary_period_s{30.0f};   // 0 = no periodic summary
    };

    MotorHealthRules() = default;
    explicit MotorHealthRules(const Params& p) : p_(p) {}
    void configure(const Params& p) { p_ = p; reset(); }
    const Params& params() const { return p_; }

    // Any pointer may be null (stream absent). dt_s = time since last step.
    std::vector<std::string> step(const UnitreeState* body, const MotorCommand* cmd,
                                  const HandState* left, const HandState* right, double dt_s) {
        std::vector<std::string> out;
        if (!p_.enabled) return out;

        if (body) {
            for (int i = 0; i < kNumMotors; ++i)
                step_body_motor(i, body->motors[i], cmd, dt_s, out);
        }
        if (left)  step_hand(0, *left,  dt_s, out);
        if (right) step_hand(1, *right, dt_s, out);

        if (p_.summary_period_s > 0) {
            summary_s_ += dt_s;
            if (summary_s_ >= p_.summary_period_s) {
                summary_s_ = 0;
                out.push_back(summary(body, left, right));
            }
        }
        return out;
    }

    void reset() {
        body_.fill(BodyTrack{});
        hand_.fill(HandTrack{});
        summary_s_ = 0;
    }

    int active_faults() const {
        int n = 0;
        for (const auto& b : body_) n += (b.error_last != 0) + b.unresp + b.frozen + (b.casing_level == 2) + (b.winding_level == 2);
        for (const auto& h : hand_) {
            n += (h.error_last != 0);
            for (int i = 0; i < kHandMotors; ++i) n += (h.casing_level[i] == 2) + (h.winding_level[i] == 2);
        }
        return n;
    }

private:
    struct BodyTrack {
        uint32_t error_last{0};
        int      casing_level{0};     // 0 ok, 1 warn, 2 critical
        int      winding_level{0};
        float    unresp_s{0};
        bool     unresp{false};
        float    frozen_s{0};
        bool     frozen{false};
        double   q_last{0};
        float    cmd_at_freeze_start{0};
        bool     have_last{false};
    };
    struct HandTrack {
        uint32_t error_last{0};
        std::array<int, kHandMotors> casing_level{};
        std::array<int, kHandMotors> winding_level{};
    };

    static std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2))) {
        char buf[512];
        va_list ap; va_start(ap, f);
        std::vsnprintf(buf, sizeof buf, f, ap);
        va_end(ap);
        return buf;
    }

    // Returns the new level and pushes a line on change.
    int temp_transition(int level, int temp, int warn, int critical, const char* sensor,
                        const char* who, std::vector<std::string>& out) {
        int next = level;
        if (temp >= critical)          next = 2;
        else if (temp >= warn)         next = (level == 2 && temp >= critical - 5) ? 2 : 1;
        else if (temp < warn - 5)      next = 0;
        if (next == level) return level;
        if (next == 2)
            out.push_back(fmt("[MotorHealth] EMERGENCY %s: %s temperature %dC >= %dC — within 5%% of the SDK cut-out; "
                              "the robot will shut this motor down on its own very soon", who, sensor, temp, critical));
        else if (next == 1)
            out.push_back(fmt("[MotorHealth] WARNING %s: %s temperature %dC >= %dC — running hot, watch the trend",
                              who, sensor, temp, warn));
        else
            out.push_back(fmt("[MotorHealth] %s: %s temperature back to %dC", who, sensor, temp));
        return next;
    }

    void step_body_motor(int i, const MotorState& m, const MotorCommand* cmd, double dt_s,
                         std::vector<std::string>& out) {
        auto& t = body_[i];
        std::string who = fmt("motor %d %s", i, g1_motor_name(i));

        // FAULT word
        if (m.error != t.error_last) {
            if (m.error != 0)
                out.push_back(fmt("[MotorHealth] EMERGENCY %s: fault word 0x%08x (SDK motorstate; decode in the Unitree app) "
                                  "— q=%.3f dq=%.3f tau=%.2f casing %dC winding %dC. The controller still receives commands "
                                  "but this motor may not execute them, and its reported state may be stale.",
                                  who.c_str(), m.error, m.q, m.dq, m.tau, m.temp_casing, m.temp_winding));
            else
                out.push_back(fmt("[MotorHealth] %s: fault cleared (was 0x%08x)", who.c_str(), t.error_last));
            t.error_last = m.error;
        }

        // TEMPERATURE (two sensors, two limits)
        t.casing_level  = temp_transition(t.casing_level,  m.temp_casing,  p_.casing_warn_c,  p_.casing_critical_c,  "casing",  who.c_str(), out);
        t.winding_level = temp_transition(t.winding_level, m.temp_winding, p_.winding_warn_c, p_.winding_critical_c, "winding", who.c_str(), out);

        // NOT RESPONDING: commanded torque not being produced
        if (cmd) {
            double expected = cmd->kp[i] * (cmd->q_target[i] - m.q) + cmd->kd[i] * (cmd->dq_target[i] - m.dq)
                            + cmd->tau_ff[i];
            bool cond = std::fabs(expected) > p_.unresp_tau_min &&
                        std::fabs(m.tau) < p_.unresp_tau_ratio * std::fabs(expected);
            t.unresp_s = cond ? t.unresp_s + static_cast<float>(dt_s) : 0.0f;
            if (!t.unresp && t.unresp_s >= p_.unresp_time_s) {
                t.unresp = true;
                out.push_back(fmt("[MotorHealth] EMERGENCY %s: NOT RESPONDING — command asks for %.1f Nm "
                                  "(kp=%.0f, q_cmd=%.3f, q=%.3f, dq=%.3f) but tau_est is %.1f Nm for %.1fs. "
                                  "The motor is not producing torque: dead driver or power loss.",
                                  who.c_str(), expected, cmd->kp[i], cmd->q_target[i], m.q, m.dq, m.tau,
                                  p_.unresp_time_s));
            } else if (t.unresp && !cond) {
                t.unresp = false;
                out.push_back(fmt("[MotorHealth] %s: producing torque again (tau_est %.1f Nm of %.1f expected)",
                                  who.c_str(), m.tau, expected));
            }
        } else {
            t.unresp_s = 0;
        }

        // STATE FROZEN
        if (t.have_last) {
            bool same = (m.q == t.q_last);
            if (same) {
                if (t.frozen_s == 0 && cmd) t.cmd_at_freeze_start = cmd->q_target[i];
                t.frozen_s += static_cast<float>(dt_s);
            } else {
                t.frozen_s = 0;
            }
            bool cmd_moved = cmd && std::fabs(cmd->q_target[i] - t.cmd_at_freeze_start) > p_.frozen_cmd_move;
            if (!t.frozen && t.frozen_s >= p_.frozen_time_s && cmd_moved) {
                t.frozen = true;
                out.push_back(fmt("[MotorHealth] EMERGENCY %s: STATE FROZEN — q=%.4f bit-identical for %.1fs while the "
                                  "command moved %.3f rad. The motor stopped reporting: the policy is observing a stale "
                                  "value, not the real joint.",
                                  who.c_str(), m.q, t.frozen_s, cmd->q_target[i] - t.cmd_at_freeze_start));
            } else if (t.frozen && !same) {
                t.frozen = false;
                out.push_back(fmt("[MotorHealth] %s: state updating again", who.c_str()));
            }
        }
        t.q_last = m.q;
        t.have_last = true;
    }

    void step_hand(int side, const HandState& h, double, std::vector<std::string>& out) {
        auto& t = hand_[side];
        const char* name = side == 0 ? "left hand" : "right hand";
        if (h.error != t.error_last) {
            if (h.error != 0)
                out.push_back(fmt("[MotorHealth] EMERGENCY %s: fault word 0x%08x (Dex3 error[0]) — bus %.1fA. "
                                  "Finger commands may not execute.", name, h.error, h.power_a));
            else
                out.push_back(fmt("[MotorHealth] %s: fault cleared (was 0x%08x)", name, t.error_last));
            t.error_last = h.error;
        }
        for (int i = 0; i < kHandMotors; ++i) {
            std::string who = fmt("%s motor %d", name, i);
            t.casing_level[i]  = temp_transition(t.casing_level[i],  h.motors[i].temp_casing,  p_.casing_warn_c,  p_.casing_critical_c,  "casing",  who.c_str(), out);
            t.winding_level[i] = temp_transition(t.winding_level[i], h.motors[i].temp_winding, p_.winding_warn_c, p_.winding_critical_c, "winding", who.c_str(), out);
        }
    }

    std::string summary(const UnitreeState* body, const HandState* l, const HandState* r) const {
        std::string s = "[MotorHealth] ";
        if (body) {
            int hc = 0, hw = 0;
            for (int i = 1; i < kNumMotors; ++i) {
                if (body->motors[i].temp_casing  > body->motors[hc].temp_casing)  hc = i;
                if (body->motors[i].temp_winding > body->motors[hw].temp_winding) hw = i;
            }
            s += fmt("hottest casing motor %d %s %dC, winding motor %d %s %dC",
                     hc, g1_motor_name(hc), body->motors[hc].temp_casing,
                     hw, g1_motor_name(hw), body->motors[hw].temp_winding);
        } else {
            s += "body state: none";
        }
        auto hand_max = [](const HandState& h) {
            int m = 0; for (const auto& x : h.motors) m = std::max<int>(m, x.temp_casing); return m;
        };
        s += fmt("; hand casing L %s R %s", l ? fmt("%dC", hand_max(*l)).c_str() : "-",
                                             r ? fmt("%dC", hand_max(*r)).c_str() : "-");
        int n = active_faults();
        s += n ? fmt("; active faults: %d", n) : "; faults: none";
        return s;
    }

    Params p_{};
    std::array<BodyTrack, kNumMotors> body_{};
    std::array<HandTrack, 2>          hand_{};
    double summary_s_{0};
};

} // namespace kist
