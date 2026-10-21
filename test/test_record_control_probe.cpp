// Injection test for the VR recording gestures: TapDetector timing
// (tap / double / dead band / hold / void) and the record_decide table.
// Header-only, no hardware.
//
// Run: ./build/test_record_control_probe   (exit 0 = all rules hold)

#include "collector/record_control.hpp"
#include "motion/tap_detector.hpp"

#include <cstdio>
#include <vector>

using kist::TapDetector;
using E = TapDetector::Event;
using kist::RecordAction; using kist::RecordState; using kist::TapButton; using kist::TapKind;

static int failures = 0;
static void check(const char* what, bool ok) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", what);
    if (!ok) ++failures;
}

// Drive a detector with a press/release script at 500 Hz; returns events in order.
struct Step { bool pressed; double seconds; bool voided = false; };
static std::vector<E> run(TapDetector& d, const std::vector<Step>& script) {
    std::vector<E> out; const double dt = 0.002;
    for (const auto& s : script)
        for (int k = 0; k < int(s.seconds / dt + 0.5); ++k) {
            E e = d.update(s.pressed, s.voided, dt);
            if (e != E::None) out.push_back(e);
        }
    return out;
}
static bool only(const std::vector<E>& v, E e) { return v.size() == 1 && v[0] == e; }

int main() {
    // ── single tap: confirmed after the gap window ──
    { TapDetector d; auto ev = run(d, {{true, 0.10}, {false, 0.50}});
      check("single: one tap + silence -> Single", only(ev, E::Single)); }
    { TapDetector d; run(d, {{true, 0.10}, {false, 0.30}});
      auto late = run(d, {{false, 0.10}});
      check("single: confirmed only after 0.35 s gap", only(late, E::Single)); }
    // ── double tap: confirmed after the window following the second release ──
    { TapDetector d; auto ev = run(d, {{true, 0.08}, {false, 0.15}, {true, 0.08}, {false, 0.60}});
      check("double: two quick taps -> Double only", only(ev, E::Double)); }
    { TapDetector d; run(d, {{true, 0.08}, {false, 0.15}, {true, 0.08}, {false, 0.30}});
      auto late = run(d, {{false, 0.10}});
      check("double: confirmed only after the 0.35 s window", only(late, E::Double)); }
    // ── triple tap: reported at the third release ──
    { TapDetector d; auto ev = run(d, {{true, 0.08}, {false, 0.15}, {true, 0.08}, {false, 0.15}, {true, 0.08}, {false, 0.60}});
      check("triple: three quick taps -> Triple only", only(ev, E::Triple)); }
    { TapDetector d; auto ev = run(d, {{true, 0.08}, {false, 0.15}, {true, 0.08}, {false, 0.15}, {true, 0.08}, {false, 0.05}, {true, 0.08}, {false, 0.60}});
      check("four taps: Triple then the fourth starts a new Single", ev.size() == 2 && ev[0] == E::Triple && ev[1] == E::Single); }
    { TapDetector d; auto ev = run(d, {{true, 0.08}, {false, 0.34}, {true, 0.08}, {false, 0.60}});
      check("double: gap 0.34 s still a double", only(ev, E::Double)); }
    { TapDetector d; auto ev = run(d, {{true, 0.08}, {false, 0.40}, {true, 0.08}, {false, 0.60}});
      check("two singles: gap 0.40 s -> Single, Single", ev.size() == 2 && ev[0] == E::Single && ev[1] == E::Single); }
    // ── dead band and hold ──
    { TapDetector d; auto ev = run(d, {{true, 0.50}, {false, 0.60}});
      check("dead band: 0.5 s press -> nothing", ev.empty()); }
    { TapDetector d; auto ev = run(d, {{true, 1.20}, {false, 0.60}});
      check("hold: 1.2 s press -> nothing here (hold gesture elsewhere)", ev.empty()); }
    { TapDetector d; auto ev = run(d, {{true, 0.08}, {false, 0.15}, {true, 0.50}, {false, 0.60}});
      check("tap then long press: cancels, no Single/Double", ev.empty()); }
    { TapDetector d; auto ev = run(d, {{true, 0.08}, {false, 0.15}, {true, 0.08}, {false, 0.15}, {true, 0.50}, {false, 0.60}});
      check("two taps then long press: cancels, no Double", ev.empty()); }
    // ── void by another button ──
    { TapDetector d; auto ev = run(d, {{true, 0.05}, {true, 0.05, true}, {false, 0.60}});
      check("void: another button during the press -> nothing", ev.empty()); }
    { TapDetector d; auto ev = run(d, {{true, 0.08}, {false, 0.10}, {false, 0.05, true}, {false, 0.60}});
      check("void: another button during the gap cancels the pending tap", ev.empty()); }
    { TapDetector d; run(d, {{true, 0.05}, {true, 0.05, true}, {false, 0.30}});
      auto ev = run(d, {{true, 0.08}, {false, 0.60}});
      check("void clears once everything is released", only(ev, E::Single)); }
    // ── e-stop windup: 4 buttons arriving over ~100 ms never read as taps ──
    { TapDetector d; auto ev = run(d, {{true, 0.03}, {true, 1.20, true}, {false, 0.05, true}, {false, 0.60}});
      check("e-stop windup voids the first-arriving button", ev.empty()); }

    // ── decision table ──
    auto dec = kist::record_decide;
    check("idle + B tap -> start",             dec(RecordState::Idle, TapButton::B, TapKind::Single) == RecordAction::Start);
    check("idle + B double -> none",           dec(RecordState::Idle, TapButton::B, TapKind::Double) == RecordAction::None);
    check("recording + B double -> stop",      dec(RecordState::Recording, TapButton::B, TapKind::Double) == RecordAction::Stop);
    check("recording + B tap -> none",         dec(RecordState::Recording, TapButton::B, TapKind::Single) == RecordAction::None);
    check("awaiting + A tap -> success",       dec(RecordState::AwaitingLabel, TapButton::A, TapKind::Single) == RecordAction::LabelSuccess);
    check("awaiting + A double -> fail",       dec(RecordState::AwaitingLabel, TapButton::A, TapKind::Double) == RecordAction::LabelFail);
    check("awaiting + A triple -> partial",    dec(RecordState::AwaitingLabel, TapButton::A, TapKind::Triple) == RecordAction::LabelPartial);
    check("recording + B triple -> none",      dec(RecordState::Recording, TapButton::B, TapKind::Triple) == RecordAction::None);
    check("awaiting + B tap -> start (unlabeled)", dec(RecordState::AwaitingLabel, TapButton::B, TapKind::Single) == RecordAction::Start);
    check("idle + A tap -> none (just IDLE)",  dec(RecordState::Idle, TapButton::A, TapKind::Single) == RecordAction::None);
    check("recording + A tap -> none",         dec(RecordState::Recording, TapButton::A, TapKind::Single) == RecordAction::None);
    check("unknown + B tap -> start (treated idle)", dec(RecordState::Unknown, TapButton::B, TapKind::Single) == RecordAction::Start);

    // ── optimistic mirror: what the commander assumes after sending ──
    auto nxt = kist::record_next_state;
    check("after start -> recording",          nxt(RecordState::Idle, RecordAction::Start) == RecordState::Recording);
    check("after stop -> awaiting_label",      nxt(RecordState::Recording, RecordAction::Stop) == RecordState::AwaitingLabel);
    check("after success -> idle",             nxt(RecordState::AwaitingLabel, RecordAction::LabelSuccess) == RecordState::Idle);
    check("after fail -> idle",                nxt(RecordState::AwaitingLabel, RecordAction::LabelFail) == RecordState::Idle);
    check("after partial -> idle",             nxt(RecordState::AwaitingLabel, RecordAction::LabelPartial) == RecordState::Idle);
    check("none keeps state",                  nxt(RecordState::Recording, RecordAction::None) == RecordState::Recording);
    // silent collector, full cycle on the mirror alone
    RecordState s = RecordState::Idle;
    auto step = [&](TapButton b, TapKind k) { auto a = dec(s, b, k); s = nxt(s, a); return a; };
    check("mirror: B tap starts",              step(TapButton::B, TapKind::Single) == RecordAction::Start);
    check("mirror: B double stops",            step(TapButton::B, TapKind::Double) == RecordAction::Stop);
    check("mirror: A tap labels success",      step(TapButton::A, TapKind::Single) == RecordAction::LabelSuccess);
    check("mirror: back to idle",              s == RecordState::Idle);

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
