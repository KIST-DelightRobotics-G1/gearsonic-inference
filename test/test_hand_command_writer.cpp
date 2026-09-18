// Verifies the grip/trigger -> Dex3-1 finger mapping without touching DDS.
// - motor 3..4 (index) driven by trigger
// - motor 0..2 (thumb) and 5..6 (middle) driven by grip
// - grip alone = pointing (index open, thumb + middle closed)
// - endpoints come from kDex3{Left,Right}{Open,Close} (empirical, real hand)
// Live wiring (VR + real robot) is checked from the main binary, not here.

#include "unitree/hand_command.hpp"

#include <cmath>
#include <cstdio>

namespace {

kist::HandCommand from_grip_and_trigger(double grip, double trigger, bool is_left) {
    grip    = std::fmax(0.0, std::fmin(1.0, grip));
    trigger = std::fmax(0.0, std::fmin(1.0, trigger));
    kist::HandCommand cmd;
    for (int i = 0; i < kist::kMotorEnd; ++i) {
        float open_q  = is_left ? kist::kDex3LeftOpen[i]  : kist::kDex3RightOpen[i];
        float close_q = is_left ? kist::kDex3LeftClose[i] : kist::kDex3RightClose[i];
        double t = kist::motor_on_trigger(i) ? trigger : grip;
        cmd.q[i] = open_q + static_cast<float>(t) * (close_q - open_q);
    }
    return cmd;
}

bool near_equal(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

int check(const char* label, bool ok) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", label);
    return ok ? 0 : 1;
}

} // namespace

int main() {
    int fails = 0;

    // ── input=0 -> open pose ──
    {
        auto l = from_grip_and_trigger(0.0, 0.0, /*is_left=*/true);
        auto r = from_grip_and_trigger(0.0, 0.0, /*is_left=*/false);
        for (int i = 0; i < kist::kMotorEnd; ++i) {
            fails += check("left  open  matches endpoint", near_equal(l.q[i], kist::kDex3LeftOpen[i]));
            fails += check("right open  matches endpoint", near_equal(r.q[i], kist::kDex3RightOpen[i]));
        }
    }

    // ── input=1 -> close pose ──
    {
        auto l = from_grip_and_trigger(1.0, 1.0, /*is_left=*/true);
        auto r = from_grip_and_trigger(1.0, 1.0, /*is_left=*/false);
        for (int i = 0; i < kist::kMotorEnd; ++i) {
            fails += check("left  close matches endpoint", near_equal(l.q[i], kist::kDex3LeftClose[i]));
            fails += check("right close matches endpoint", near_equal(r.q[i], kist::kDex3RightClose[i]));
        }
    }

    // ── fingers all open at q=0 (empirical: 0 = extended = open) ──
    {
        auto lo = from_grip_and_trigger(0.0, 0.0, /*is_left=*/true);
        auto ro = from_grip_and_trigger(0.0, 0.0, /*is_left=*/false);
        for (int i = kist::kIndexBegin; i < kist::kMotorEnd; ++i) {
            fails += check("left  finger open  == 0", near_equal(lo.q[i], 0.0f));
            fails += check("right finger open  == 0", near_equal(ro.q[i], 0.0f));
        }
    }

    // ── index curls to the signed extreme at trigger=1 ──
    {
        auto lc = from_grip_and_trigger(0.0, 1.0, /*is_left=*/true);
        auto rc = from_grip_and_trigger(0.0, 1.0, /*is_left=*/false);
        for (int i = kist::kIndexBegin; i < kist::kMiddleBegin; ++i) {
            fails += check("left  index close is negative", lc.q[i] < 0.0f);
            fails += check("right index close is positive", rc.q[i] > 0.0f);
        }
    }

    // ── pointing: grip alone closes thumb + middle, index stays extended ──
    {
        auto lp = from_grip_and_trigger(1.0, 0.0, /*is_left=*/true);
        auto rp = from_grip_and_trigger(1.0, 0.0, /*is_left=*/false);
        for (int i = kist::kThumbBegin; i < kist::kIndexBegin; ++i) {
            fails += check("point: left  thumb closed",  near_equal(lp.q[i], kist::kDex3LeftClose[i]));
            fails += check("point: right thumb closed",  near_equal(rp.q[i], kist::kDex3RightClose[i]));
        }
        for (int i = kist::kIndexBegin; i < kist::kMiddleBegin; ++i) {
            fails += check("point: left  index open",    near_equal(lp.q[i], 0.0f));
            fails += check("point: right index open",    near_equal(rp.q[i], 0.0f));
        }
        for (int i = kist::kMiddleBegin; i < kist::kMotorEnd; ++i) {
            fails += check("point: left  middle closed", near_equal(lp.q[i], kist::kDex3LeftClose[i]));
            fails += check("point: right middle closed", near_equal(rp.q[i], kist::kDex3RightClose[i]));
        }
    }

    // ── axis independence: trigger moves only the index, grip only thumb + middle ──
    {
        auto base   = from_grip_and_trigger(0.5, 0.5, /*is_left=*/true);
        auto more_g = from_grip_and_trigger(0.9, 0.5, /*is_left=*/true);
        auto more_t = from_grip_and_trigger(0.5, 0.9, /*is_left=*/true);
        for (int i = 0; i < kist::kMotorEnd; ++i) {
            if (kist::motor_on_trigger(i)) {
                fails += check("trigger moves index",   !near_equal(base.q[i], more_t.q[i]));
                fails += check("grip leaves index",      near_equal(base.q[i], more_g.q[i]));
            } else if (i != kist::kThumbBegin) {  // motor 0 (thumb rotation) is pinned at center
                fails += check("grip moves thumb/middle",  !near_equal(base.q[i], more_g.q[i]));
                fails += check("trigger leaves thumb/middle", near_equal(base.q[i], more_t.q[i]));
            }
        }
    }

    // ── clamp ──
    {
        auto neg = from_grip_and_trigger(-0.5, -0.5, /*is_left=*/true);
        auto big = from_grip_and_trigger( 2.0,  2.0, /*is_left=*/true);
        for (int i = 0; i < kist::kMotorEnd; ++i) {
            fails += check("clamp low  == open",  near_equal(neg.q[i], kist::kDex3LeftOpen[i]));
            fails += check("clamp high == close", near_equal(big.q[i], kist::kDex3LeftClose[i]));
        }
    }

    std::printf("\n%s (%d failures)\n", fails == 0 ? "ALL PASS" : "FAIL", fails);
    return fails == 0 ? 0 : 1;
}
