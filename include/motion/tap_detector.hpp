#pragma once

namespace kist {

// Short-press detector for one face button, on top of the existing 1 s
// hold gestures. Feed it every tick (500 Hz):
//   - a press released within kTapMaxS is a TAP; a press held longer is
//     nothing here (0.3..1 s is a dead band, >= 1 s is the hold gesture);
//   - two taps whose gap (first release -> second press) is within
//     kDoubleGapS are a DOUBLE, reported at the second release;
//   - a lone tap is reported as SINGLE once the gap window has passed
//     with no second press (so a single is confirmed kDoubleGapS late);
//   - any tick with another face button down voids the gesture until
//     every button is released (same rule as the hold gestures).
struct TapDetector {
    static constexpr double kTapMaxS    = 0.30;   // press shorter than this = tap
    static constexpr double kDoubleGapS = 0.35;   // release -> next press within this = double

    enum class Event { None, Single, Double };

    Event update(bool pressed, bool voided, double dt_s) {
        if (voided) { void_ = true; pending_ = false; second_ = false; }
        if (void_) {
            if (!pressed && !voided) void_ = false;
            was_pressed_ = pressed;
            return Event::None;
        }
        if (pressed) {
            if (!was_pressed_) {                      // press edge
                press_s_ = 0.0;
                second_  = pending_ && since_release_s_ <= kDoubleGapS;
                pending_ = false;
            } else {
                press_s_ += dt_s;
            }
            was_pressed_ = true;
            return Event::None;
        }
        if (was_pressed_) {                           // release edge
            was_pressed_ = false;
            if (press_s_ < kTapMaxS) {
                if (second_) { second_ = false; return Event::Double; }
                pending_ = true; since_release_s_ = 0.0;
            } else {
                second_ = false;                      // long press: cancels a pending double
            }
            return Event::None;
        }
        if (pending_) {
            since_release_s_ += dt_s;
            if (since_release_s_ > kDoubleGapS) { pending_ = false; return Event::Single; }
        }
        return Event::None;
    }

    void reset() { *this = TapDetector{}; }

private:
    bool   was_pressed_{false};
    bool   pending_{false};       // a tap waiting to be confirmed single
    bool   second_{false};        // current press is the second of a double
    bool   void_{false};
    double press_s_{0.0};
    double since_release_s_{0.0};
};

} // namespace kist
