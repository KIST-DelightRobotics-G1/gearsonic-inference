// Injection test for the ControlArbiter stage — the transition function is
// pure (all inputs passed in) and the token blends are plain arithmetic on
// a private instance, so every arbitration rule and the continuity of the
// commanded-token trajectory are checked here without threads, buffers, or
// hardware.
//
// Run: ./build/test_arbiter_probe   (exit 0 = all rules hold)

#include "control/control_arbiter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using Mode = kist::ControlArbiter::Mode;

static const char* name(Mode m) {
    switch (m) {
        case Mode::kNormal:     return "kNormal";
        case Mode::kTeleop:     return "kTeleop";
        case Mode::kVla:        return "kVla";
        case Mode::kRecovering: return "kRecovering";
    }
    return "?";
}

static int failures = 0;

static void expect(const char* what, Mode cur, bool fresh, bool alive,
                   bool calib, bool rec_done, Mode want) {
    Mode got = kist::ControlArbiter::decide(cur, fresh, alive, calib, rec_done);
    if (got != want) {
        std::printf("FAIL %-55s %s -> %s (want %s)\n",
                    what, name(cur), name(got), name(want));
        ++failures;
    } else {
        std::printf("ok   %-55s %s -> %s\n", what, name(cur), name(got));
    }
}

int main() {
    // ── origin: entries ─────────────────────────────────────────
    expect("origin idle stays origin",
           Mode::kNormal, false, false, false, false, Mode::kNormal);
    expect("fresh token claims VLA",
           Mode::kNormal, true, true, false, false, Mode::kVla);
    expect("calibration claims teleop",
           Mode::kNormal, false, false, true, false, Mode::kTeleop);
    expect("both present: VLA auto-claim wins the race",
           Mode::kNormal, true, true, true, false, Mode::kVla);
    expect("alive-but-not-fresh token cannot claim",
           Mode::kNormal, false, true, false, false, Mode::kNormal);

    // ── teleop: first-come, VLA ignored ─────────────────────────
    expect("teleop holds against a fresh stream",
           Mode::kTeleop, true, true, true, false, Mode::kTeleop);
    expect("teleop off returns to origin (stream still ignored this tick)",
           Mode::kTeleop, true, true, false, false, Mode::kNormal);
    expect("teleop holds while calibrated, no stream",
           Mode::kTeleop, false, false, true, false, Mode::kTeleop);

    // ── VLA: hysteresis + exit only through recovery ────────────
    expect("VLA rides out a 200-500ms hiccup",
           Mode::kVla, false, true, false, false, Mode::kVla);
    expect("VLA holds regardless of calibration flag",
           Mode::kVla, true, true, true, false, Mode::kVla);
    expect("stream dead -> recovering, never a direct handover",
           Mode::kVla, false, false, true, false, Mode::kRecovering);

    // ── recovery: blocks everything until done ──────────────────
    expect("recovery ignores a resumed stream until done",
           Mode::kRecovering, true, true, false, false, Mode::kRecovering);
    expect("recovery ignores calibration until done",
           Mode::kRecovering, false, false, true, false, Mode::kRecovering);
    expect("recovery done -> origin",
           Mode::kRecovering, false, false, false, true, Mode::kNormal);

    // ── lifecycle: VLA loss -> origin -> re-claim, no restart ───
    Mode m = Mode::kNormal;
    m = kist::ControlArbiter::decide(m, true, true, false, false);    // claim
    m = kist::ControlArbiter::decide(m, false, false, false, false);  // lost
    m = kist::ControlArbiter::decide(m, true, true, false, true);     // done
    m = kist::ControlArbiter::decide(m, true, true, false, false);    // re-claim
    if (m != Mode::kVla) {
        std::printf("FAIL lifecycle re-claim after recovery: ends at %s\n", name(m));
        ++failures;
    } else {
        std::printf("ok   lifecycle: claim -> lost -> origin -> re-claim\n");
    }

    // ── token blend continuity: claim -> loss -> recovery -> handoff ──
    // The commanded token must never step: through the standing blend and
    // the handoff crossfade, per-tick movement stays a small fraction of
    // the total gap, and both blends land exactly on their targets.
    {
        using Arb = kist::ControlArbiter;
        Arb arb;  // private instance — the singleton stays untouched

        Arb::Token stream_token{}, encoder_token{};
        for (size_t i = 0; i < stream_token.size(); ++i) {
            stream_token[i]  = 2.0f - 0.05f * static_cast<float>(i);
            encoder_token[i] = -1.0f + 0.03f * static_cast<float>(i);
        }

        arb.step_mode(true, true, false, false);                    // claim
        Arb::Token prev = arb.select_token(nullptr, &stream_token); // = stream

        float max_gap = 0.0f;
        for (size_t i = 0; i < prev.size(); ++i)
            max_gap = std::fmax(max_gap,
                                std::fabs(prev[i] - kist::kVlaSafeStandingToken[i]));
        const float step_limit = max_gap * 0.15f;  // 50 ticks -> ~2-4%/tick

        arb.step_mode(false, false, false, false);                  // lost
        float worst_step = 0.0f;
        for (int t = 0; t < Arb::kRecoveryBlendTicks; ++t) {
            auto tok = arb.select_token(nullptr, nullptr);
            for (size_t i = 0; i < tok.size(); ++i)
                worst_step = std::fmax(worst_step, std::fabs(tok[i] - prev[i]));
            prev = tok;
        }
        bool landed = true;
        for (size_t i = 0; i < prev.size(); ++i)
            if (std::fabs(prev[i] - kist::kVlaSafeStandingToken[i]) > 1e-5f)
                landed = false;
        if (!landed || worst_step > step_limit || !arb.recovery_blend_done()) {
            std::printf("FAIL recovery blend: landed=%d worst_step=%.4f limit=%.4f done=%d\n",
                        landed, worst_step, step_limit, arb.recovery_blend_done());
            ++failures;
        } else {
            std::printf("ok   recovery blend: continuous, lands on the standing token\n");
        }

        arb.step_mode(false, false, false, true);                   // -> origin
        float handoff_gap = 0.0f;
        for (size_t i = 0; i < prev.size(); ++i)
            handoff_gap = std::fmax(handoff_gap,
                                    std::fabs(prev[i] - encoder_token[i]));
        const float handoff_limit = handoff_gap * 0.25f;  // 25 ticks

        worst_step = 0.0f;
        Arb::Token tok{};
        for (int t = 0; t <= Arb::kHandoffBlendTicks; ++t) {
            tok = arb.select_token(&encoder_token, nullptr);
            for (size_t i = 0; i < tok.size(); ++i)
                worst_step = std::fmax(worst_step, std::fabs(tok[i] - prev[i]));
            prev = tok;
        }
        bool converged = true;
        for (size_t i = 0; i < tok.size(); ++i)
            if (std::fabs(tok[i] - encoder_token[i]) > 1e-5f)
                converged = false;
        if (!converged || worst_step > handoff_limit) {
            std::printf("FAIL handoff blend: converged=%d worst_step=%.4f limit=%.4f\n",
                        converged, worst_step, handoff_limit);
            ++failures;
        } else {
            std::printf("ok   handoff blend: continuous, converges to the encoder token\n");
        }
    }

    if (failures) {
        std::printf("%d rule(s) violated\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("all arbitration rules hold\n");
    return EXIT_SUCCESS;
}
