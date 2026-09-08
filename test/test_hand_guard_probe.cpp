// Injection test for the HandGuard stage — pure arithmetic on a private
// instance, so the stall-torque bound, the stall latch/release and the
// pass-through paths are checked without DDS, threads or hardware.
//
// Run: ./build/test_hand_guard_probe   (exit 0 = all rules hold)

#include "unitree/hand_guard.hpp"

#include <cmath>
#include <cstdio>

using kist::HandCommand;
using kist::HandGuard;
using kist::HandState;

static int failures = 0;

static void check(const char* what, bool ok) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", what);
    if (!ok) ++failures;
}
static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

static HandCommand desired_left(float q_all, float kp = 1.5f, float kd = 0.1f) {
    HandCommand c;
    c.q.fill(q_all); c.kp.fill(kp); c.kd.fill(kd); c.enabled = true;
    return c;
}
static HandState meas_all(float q, float dq = 0.0f) {
    HandState s;
    for (auto& m : s.motors) { m.q = q; m.dq = dq; }
    return s;
}

int main() {
    HandGuard::Params p;
    p.kp = 5.0f; p.kd = 0.1f; p.tau_max = 1.0f;          // e_max = 0.2 rad
    p.stall_err_th = 0.15f; p.stall_vel_th = 0.05f; p.stall_time_s = 0.3f;
    p.stall_offset = 0.05f; p.kp_hold = 2.0f;
    const double dt = 0.01;
    const float e_max = p.tau_max / p.kp;

    // Motor 2 (left thumb flexion): range 0..1.75, open=0, closed=1.75.
    const int m = 2;

    // ── no measurement -> pass through unchanged ──
    {
        HandGuard g(true, p);
        auto d = desired_left(1.0f);
        auto o = g.apply(d, nullptr, dt);
        check("no meas: q passes through",  near(o.q[m], 1.0f));
        check("no meas: kp untouched",      near(o.kp[m], 1.5f));
    }

    // ── free tracking: target within e_max -> exact target, tracking gains ──
    {
        HandGuard g(true, p);
        auto s = meas_all(0.9f, 0.5f);
        auto o = g.apply(desired_left(1.0f), &s, dt);
        check("free: q_cmd == q_des",       near(o.q[m], 1.0f));
        check("free: kp = tracking kp",     near(o.kp[m], p.kp));
        check("free: kd applied",           near(o.kd[m], p.kd));
    }

    // ── blocked far away: q_cmd sits e_max ahead of q_meas ──
    {
        HandGuard g(true, p);
        auto s = meas_all(0.3f, 0.0f);
        auto o = g.apply(desired_left(1.5f), &s, dt);
        check("blocked: q_cmd = q_meas + e_max", near(o.q[m], 0.3f + e_max));
        check("blocked: stall torque = tau_max", near(p.kp * (o.q[m] - 0.3f), p.tau_max));
        auto o2 = g.apply(desired_left(0.0f), &s, dt);
        check("blocked (other side): q_cmd = q_meas - e_max", near(o2.q[m], 0.3f - e_max));
    }

    // ── stall latch after stall_time_s, then release when target returns ──
    {
        HandGuard g(true, p);
        auto s = meas_all(0.3f, 0.0f);
        int ticks_to_latch = 0;
        for (int k = 0; k < 100 && !g.latched(m); ++k) { g.apply(desired_left(1.5f), &s, dt); ++ticks_to_latch; }
        check("stall: latches",                       g.latched(m));
        check("stall: latch time ~ stall_time_s",     ticks_to_latch == 30 || ticks_to_latch == 31);
        auto o = g.apply(desired_left(1.5f), &s, dt);
        check("stall: q_cmd = q_meas + offset",       near(o.q[m], 0.3f + p.stall_offset));
        check("stall: kp = kp_hold",                  near(o.kp[m], p.kp_hold));

        // moving finger (obstacle removed) keeps the latch until target is near
        auto s2 = meas_all(0.6f, 0.4f);
        g.apply(desired_left(1.5f), &s2, dt);
        check("stall: stays latched while far",       g.latched(m));

        auto s3 = meas_all(1.4f, 0.0f);
        auto o3 = g.apply(desired_left(1.5f), &s3, dt);
        check("stall: releases within err_th",        !g.latched(m));
        check("stall: back to tracking after release", near(o3.q[m], 1.5f) && near(o3.kp[m], p.kp));
    }

    // ── moving finger never latches even with large error ──
    {
        HandGuard g(true, p);
        auto s = meas_all(0.3f, 0.5f);
        for (int k = 0; k < 100; ++k) g.apply(desired_left(1.5f), &s, dt);
        check("moving: no latch",                     !g.latched(m));
    }

    // ── URDF limit clamp on the derived command ──
    {
        HandGuard g(true, p);
        auto s = meas_all(1.70f, 0.0f);
        auto o = g.apply(desired_left(1.75f), &s, dt);
        check("limit: q_cmd <= max",                  o.q[m] <= kist::kDex3LeftMax[m] + 1e-6f);
        auto s2 = meas_all(0.05f, 0.0f);
        auto o2 = g.apply(desired_left(-1.0f), &s2, dt);
        check("limit: q_cmd >= min",                  o2.q[m] >= kist::kDex3LeftMin[m] - 1e-6f);
    }

    // ── guard disabled / stop command -> pass through ──
    {
        auto pd = p; pd.enabled = false;
        HandGuard g(true, pd);
        auto s = meas_all(0.3f, 0.0f);
        auto o = g.apply(desired_left(1.5f), &s, dt);
        check("disabled: pass through",               near(o.q[m], 1.5f) && near(o.kp[m], 1.5f));

        HandGuard g2(true, p);
        auto stop = desired_left(0.0f, 0.0f, 0.0f); stop.enabled = false;
        auto o2 = g2.apply(stop, &s, dt);
        check("stop cmd: pass through",               near(o2.q[m], 0.0f) && near(o2.kp[m], 0.0f));
    }

    // ── temperature latch is immediate ──
    {
        auto pt = p; pt.temp_max_c = 60;
        HandGuard g(true, pt);
        auto s = meas_all(0.3f, 0.0f);
        for (auto& mm : s.motors) mm.temperature = 65;
        auto o = g.apply(desired_left(1.5f), &s, dt);
        check("hot: latches on first tick",           g.latched(m) && near(o.kp[m], p.kp_hold));
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
