// Injection test for SmplResampler — pure arithmetic on synthetic samples,
// so the grid interpolation, the adaptive delay and the late-data hold are
// checked without the headset, threads or hardware.
//
// Run: ./build/test_smpl_resample_probe   (exit 0 = all rules hold)

#include "teleop/smpl_resampler.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

using kist::SmplPose;
using kist::SmplResampler;
using clock_t_ = std::chrono::steady_clock;
using ms = std::chrono::milliseconds;

static int failures = 0;
static void check(const char* what, bool ok) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", what);
    if (!ok) ++failures;
}
static bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

// A pose whose every joint x equals `x`, root yaw = `yaw` rad, wrist[0] = x.
static SmplPose pose_at(double x, double yaw = 0.0) {
    SmplPose p{};
    for (auto& j : p.joints_local) j = {x, 2.0 * x, -x};
    p.wrist_joint_pos = {x, 0, 0, 0, 0, 0};
    p.anchor_quat = {std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)};
    return p;
}

int main() {
    const auto T0 = clock_t_::time_point{} + ms(1000);

    // ── lerp endpoints and midpoint ──
    {
        auto a = pose_at(0.0), b = pose_at(1.0, 0.2);
        auto m = SmplResampler::lerp(a, b, 0.5);
        check("lerp: alpha 0 is a",        near(SmplResampler::lerp(a, b, 0.0).joints_local[5][0], 0.0));
        check("lerp: alpha 1 is b",        near(SmplResampler::lerp(a, b, 1.0).joints_local[5][1], 2.0));
        check("lerp: midpoint positions",  near(m.joints_local[23][0], 0.5) && near(m.joints_local[23][2], -0.5));
        check("lerp: midpoint wrist q",    near(m.wrist_joint_pos[0], 0.5));
        double yaw = 2.0 * std::atan2(m.anchor_quat[3], m.anchor_quat[0]);
        check("lerp: quaternion halfway",  near(yaw, 0.1, 1e-6));
        auto bneg = b; for (auto& q : bneg.anchor_quat) q = -q;   // same rotation, flipped sign
        auto m2 = SmplResampler::lerp(a, bneg, 0.5);
        double yaw2 = 2.0 * std::atan2(m2.anchor_quat[3], m2.anchor_quat[0]);
        check("lerp: quaternion sign-safe", near(std::fabs(yaw2), 0.1, 1e-6));
    }

    // ── alpha_for: clamped position between the two samples ──
    {
        auto t0 = T0, t1 = T0 + ms(40);
        check("alpha: at t0 -> 0",       near(SmplResampler::alpha_for(t0, t1, t0), 0.0));
        check("alpha: quarter",          near(SmplResampler::alpha_for(t0, t1, t0 + ms(10)), 0.25));
        check("alpha: past t1 clamps 1", near(SmplResampler::alpha_for(t0, t1, t1 + ms(30)), 1.0));
        check("alpha: before t0 clamps 0", near(SmplResampler::alpha_for(t0, t1, t0 - ms(5)), 0.0));
        check("alpha: zero span -> 1",   near(SmplResampler::alpha_for(t0, t0, t0), 1.0));
    }

    // ── grid sampling on a 30Hz stream (33.3 ms) with the default params ──
    {
        SmplResampler r;
        SmplPose out;
        check("sample: empty -> false", !r.sample(T0, out));
        // samples: x = 0,1,2,... every 33 ms — a constant-velocity sweep
        for (int k = 0; k < 20; ++k)
            r.push(pose_at(double(k)), T0 + ms(33 * k));
        check("sample: interval learned ~33ms", std::fabs(r.interval_s() - 0.033) < 0.002);
        double delay = r.delay_s();
        check("sample: delay = 1.5x interval", std::fabs(delay - 1.5 * r.interval_s()) < 1e-9);
        // newest sample at k=19 (t = 627 ms). Query at now = t19 + 20 ms: target
        // = now - delay lies between samples 18 and 19 -> x between 18 and 19
        auto now = T0 + ms(33 * 19 + 20);
        check("sample: returns a pose", r.sample(now, out));
        double x = out.joints_local[0][0];
        double target_s = (33.0 * 19 + 20) / 1000.0 - delay;   // seconds since T0
        double expect   = target_s / 0.033;                     // constant velocity
        check("sample: interpolated x matches constant velocity", std::fabs(x - expect) < 0.05);
        check("sample: lies between the two newest samples", x >= 18.0 && x <= 19.0);
        // late data: much later than the newest sample -> holds the newest
        check("sample: late data holds newest", r.sample(now + ms(500), out) && near(out.joints_local[0][0], 19.0));
    }

    // ── grid sampling is smooth: consecutive 20 ms queries advance evenly ──
    {
        SmplResampler r;
        for (int k = 0; k < 20; ++k)
            r.push(pose_at(double(k)), T0 + ms(33 * k));
        SmplPose a, b, c;
        auto now = T0 + ms(33 * 19 + 5);
        r.sample(now, a); r.sample(now + ms(20), b); r.sample(now + ms(40), c);
        double d1 = b.joints_local[0][0] - a.joints_local[0][0];
        double d2 = c.joints_local[0][0] - b.joints_local[0][0];
        check("smooth: equal steps for equal grid ticks", std::fabs(d1 - d2) < 1e-6 && d1 > 0.0);
        check("smooth: step = 20ms of a 33ms-per-unit sweep", std::fabs(d1 - 20.0 / 33.0) < 0.02);
    }

    // ── delay clamps and one-sample hold ──
    {
        SmplResampler r;
        r.push(pose_at(7.0), T0);
        SmplPose out;
        check("one sample: held", r.sample(T0 + ms(100), out) && near(out.joints_local[0][0], 7.0));
        SmplResampler slow;   // 5 Hz stream -> delay hits the 100 ms cap
        for (int k = 0; k < 6; ++k) slow.push(pose_at(k), T0 + ms(200 * k));
        check("delay: capped at max", near(slow.delay_s(), 0.100));
        SmplResampler fast;   // 200 Hz stream -> delay floors at 20 ms
        for (int k = 0; k < 40; ++k) fast.push(pose_at(k), T0 + ms(5 * k));
        check("delay: floored at min", near(fast.delay_s(), 0.020));
        slow.reset();
        check("reset: back to empty", slow.count() == 0 && !slow.sample(T0, out));
        SmplResampler dup; dup.push(pose_at(1.0), T0); dup.push(pose_at(2.0), T0);
        check("push: duplicate time ignored", dup.count() == 1);
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
