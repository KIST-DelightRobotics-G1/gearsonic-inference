// Injection test for MotorHealthRules — the detector is pure, so every
// condition, its edge-triggering and its clearing are checked here without
// threads, DDS or hardware.
//
// Run: ./build/test_motor_health_probe   (exit 0 = all rules hold)

#include "unitree/motor_health_rules.hpp"

#include <cstdio>
#include <string>

using kist::MotorHealthRules;
using kist::UnitreeState;
using kist::MotorCommand;
using kist::HandState;

static int failures = 0;
static void check(const char* what, bool ok) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", what);
    if (!ok) ++failures;
}
static bool has(const std::vector<std::string>& lines, const char* needle) {
    for (const auto& l : lines) if (l.find(needle) != std::string::npos) return true;
    return false;
}
static int run(MotorHealthRules& r, const UnitreeState& s, const MotorCommand& c, int ticks, double dt,
               std::vector<std::string>* last = nullptr) {
    int total = 0;
    for (int k = 0; k < ticks; ++k) {
        auto out = r.step(&s, &c, nullptr, nullptr, dt);
        total += static_cast<int>(out.size());
        if (last) *last = out;
    }
    return total;
}

int main() {
    MotorHealthRules::Params p;
    p.summary_period_s = 0;  // keep the summary out of the counts
    const double dt = 0.05;
    const int m = 26;  // right_wrist_roll

    UnitreeState healthy;
    for (int i = 0; i < 29; ++i) { healthy.motors[i].q = 0.1 * i; healthy.motors[i].temp_casing = 40; healthy.motors[i].temp_winding = 60; }
    MotorCommand cmd;
    for (int i = 0; i < 29; ++i) { cmd.q_target[i] = static_cast<float>(0.1 * i); cmd.kp[i] = 100.0f; }

    // ── healthy: silent ──
    {
        MotorHealthRules r(p);
        // wiggle q a little each tick so nothing looks frozen
        UnitreeState s = healthy; int n = 0;
        for (int k = 0; k < 60; ++k) { for (auto& mm : s.motors) mm.q += 1e-6; n += (int)r.step(&s, &cmd, nullptr, nullptr, dt).size(); }
        check("healthy: no output", n == 0);
    }

    // ── fault word: one line on set, one on clear ──
    {
        MotorHealthRules r(p);
        UnitreeState s = healthy; std::vector<std::string> out;
        s.motors[m].error = 0x0c;
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("fault: EMERGENCY line with name", has(out, "EMERGENCY motor 26 right_wrist_roll") && has(out, "0x0000000c"));
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("fault: not repeated", out.empty());
        s.motors[m].error = 0;
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("fault: cleared line", has(out, "fault cleared"));
    }

    // ── temperature: warn, critical, hysteresis ──
    {
        MotorHealthRules r(p);
        UnitreeState s = healthy; std::vector<std::string> out;
        s.motors[3].temp_casing = 72;
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("casing: WARNING at 72", has(out, "WARNING motor 3 left_knee: casing temperature 72C"));
        s.motors[3].temp_casing = 86;
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("casing: EMERGENCY at 86", has(out, "EMERGENCY motor 3 left_knee: casing temperature 86C >= 80C"));
        s.motors[3].temp_casing = 67;  // above warn-5 -> stays flagged
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("casing: hysteresis holds at 67", !has(out, "back to"));
        s.motors[3].temp_casing = 60;
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("casing: clears at 60", has(out, "casing temperature back to 60C"));
        // winding runs hotter by design: 100C is silent, 106 warns, 121 is critical
        s.motors[3].temp_winding = 100;
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("winding: 100C silent", out.empty());
        s.motors[3].temp_winding = 106;
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("winding: WARNING at 106", has(out, "winding temperature 106C >= 102C"));
        s.motors[3].temp_winding = 121;
        out = r.step(&s, &cmd, nullptr, nullptr, dt);
        check("winding: EMERGENCY at 121", has(out, "EMERGENCY motor 3 left_knee: winding temperature 121C >= 114C"));
    }

    // ── not responding: torque asked for but not produced ──
    {
        // A standing leg: target 0.8 rad away at kp 29 asks ~23 Nm and the
        // live motor reports it -> silent (this is the normal RL regime).
        MotorHealthRules r(p);
        UnitreeState s = healthy; MotorCommand c = cmd;
        c.kp[4] = 29.0f; c.q_target[4] = static_cast<float>(s.motors[4].q + 0.8);
        s.motors[4].tau = 29.0 * 0.8 * 0.9;   // within 10% of expected
        int n = 0;
        for (int k = 0; k < 60; ++k) { for (auto& mm : s.motors) mm.q += 1e-6; n += (int)r.step(&s, &c, nullptr, nullptr, dt).size(); }
        check("unresp: loaded standing joint is silent", n == 0);

        // Same command, but tau_est ~0: the driver is not producing -> fires after 1s
        MotorHealthRules r2(p);
        UnitreeState s2 = healthy; s2.motors[4].tau = 0.5;
        std::vector<std::string> out; int ticks = 0; int lines = 0;
        for (; ticks < 100; ++ticks) {
            for (auto& mm : s2.motors) mm.q += 1e-6;
            out = r2.step(&s2, &c, nullptr, nullptr, dt);
            lines += (int)out.size();
            if (has(out, "NOT RESPONDING")) break;
        }
        check("unresp: dead driver fires",        has(out, "motor 4 left_ankle_pitch: NOT RESPONDING") && has(out, "tau_est is 0.5 Nm"));
        check("unresp: after ~1s",                ticks >= 19 && ticks <= 21);
        check("unresp: single line so far",       lines == 1);
        s2.motors[4].tau = 20.0;   // torque comes back -> clears
        out = r2.step(&s2, &c, nullptr, nullptr, dt);
        check("unresp: clears when torque returns", has(out, "producing torque again"));

        // Small requested torque (< tau_min) never flags, whatever tau_est says
        MotorHealthRules r3(p);
        UnitreeState s3 = healthy; MotorCommand c3 = cmd;
        c3.q_target[m] = static_cast<float>(s3.motors[m].q + 0.02);   // ~2 Nm at kp 100
        n = 0;
        for (int k = 0; k < 60; ++k) { for (auto& mm : s3.motors) mm.q += 1e-6; n += (int)r3.step(&s3, &c3, nullptr, nullptr, dt).size(); }
        check("unresp: tiny request is silent",   n == 0);

        // Damping command (kp=0, kd only, dq=0) asks for nothing -> silent
        MotorHealthRules r4(p);
        MotorCommand c4; for (int i = 0; i < 29; ++i) c4.kd[i] = 8.0f;
        UnitreeState s4 = healthy;
        n = 0;
        for (int k = 0; k < 60; ++k) { for (auto& mm : s4.motors) mm.q += 1e-6; n += (int)r4.step(&s4, &c4, nullptr, nullptr, dt).size(); }
        check("unresp: damping is silent",        n == 0);
    }

    // ── frozen: q bit-identical while the command moves ──
    {
        MotorHealthRules r(p);
        UnitreeState s = healthy; MotorCommand c = cmd;
        c.q_target[m] = static_cast<float>(s.motors[m].q);  // starts on target
        std::vector<std::string> out; int fired_at = -1;
        for (int k = 0; k < 60; ++k) {
            c.q_target[m] += 0.005f;   // command sweeps away; q never changes
            for (int i = 0; i < 29; ++i) if (i != m) s.motors[i].q += 1e-6;
            out = r.step(&s, &c, nullptr, nullptr, dt);
            if (has(out, "STATE FROZEN")) { fired_at = k; break; }
        }
        check("frozen: fires", fired_at >= 0);
        check("frozen: after ~1s", fired_at >= 19 && fired_at <= 22);
        // a still robot (command not moving) must NOT be flagged as frozen
        MotorHealthRules r2(p); UnitreeState s2 = healthy; MotorCommand c2 = cmd;
        check("frozen: still robot is silent", run(r2, s2, c2, 60, dt) == 0);
    }

    // ── hands: fault word + temperature ──
    {
        MotorHealthRules r(p);
        HandState l; l.error = 0x2; l.power_a = 1.5f;
        auto out = r.step(nullptr, nullptr, &l, nullptr, dt);
        check("hand: fault line", has(out, "EMERGENCY left hand: fault word 0x00000002"));
        HandState rr; rr.motors[4].temp_casing = 90;
        out = r.step(nullptr, nullptr, nullptr, &rr, dt);
        check("hand: temp EMERGENCY", has(out, "EMERGENCY right hand motor 4: casing temperature 90C"));
    }

    // ── summary line ──
    {
        auto ps = p; ps.summary_period_s = 0.1f;
        MotorHealthRules r(ps);
        UnitreeState s = healthy; s.motors[9].temp_casing = 55; s.motors[20].temp_winding = 80;
        std::vector<std::string> out;
        run(r, s, cmd, 4, dt, &out);
        check("summary: hottest named", has(out, "hottest casing motor 9 right_knee 55C, winding motor 20 left_wrist_pitch 80C"));
    }

    // ── disabled: silent ──
    {
        auto pd = p; pd.enabled = false;
        MotorHealthRules r(pd);
        UnitreeState s = healthy; s.motors[m].error = 0xff; s.motors[m].temp_casing = 120;
        check("disabled: silent", r.step(&s, &cmd, nullptr, nullptr, dt).empty());
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
