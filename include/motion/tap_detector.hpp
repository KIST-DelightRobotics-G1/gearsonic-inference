#pragma once

namespace kist {

// Short-press detector for one face button, on top of the existing 1 s
// hold gestures. Feed it every tick (500 Hz):
//   - a press released within kTapMaxS is a TAP; a press held longer is
//     nothing here (0.3..1 s is a dead band, >= 1 s is the hold gesture)
//     and cancels any taps counted so far;
//   - taps whose gap (release -> next press) stays within kDoubleGapS are
//     counted together: the count is reported once the window after the
//     last tap passes with no further press — SINGLE, DOUBLE or TRIPLE
//     (a third tap is reported at once; more than three is void);
//   - any tick with another face button down voids the gesture until
//     every button is released (same rule as the hold gestures).
struct TapDetector {
    static constexpr double kTapMaxS    = 0.30;   // press shorter than this = tap
    static constexpr double kDoubleGapS = 0.35;   // release -> next press within this = same gesture
    static constexpr int    kMaxTaps    = 3;

    enum class Event { None, Single, Double, Triple };

    Event update(bool pressed, bool voided, double dt_s) {
        if (voided) { void_ = true; count_ = 0; pending_ = false; }
        if (void_) {
            if (!pressed && !voided) void_ = false;
            was_pressed_ = pressed;
            return Event::None;
        }
        if (pressed) {
            if (!was_pressed_) {                      // press edge
                press_s_ = 0.0;
                if (!(pending_ && since_release_s_ <= kDoubleGapS)) count_ = 0;   // too late: new gesture
                pending_ = false;
            } else {
                press_s_ += dt_s;
            }
            was_pressed_ = true;
            return Event::None;
        }
        if (was_pressed_) {                           // release edge
            was_pressed_ = false;
            if (press_s_ >= kTapMaxS) { count_ = 0; return Event::None; }   // long press cancels
            ++count_;
            if (count_ >= kMaxTaps) { count_ = 0; return Event::Triple; }
            pending_ = true; since_release_s_ = 0.0;
            return Event::None;
        }
        if (pending_) {
            since_release_s_ += dt_s;
            if (since_release_s_ > kDoubleGapS) {
                pending_ = false;
                Event e = count_ == 1 ? Event::Single : Event::Double;
                count_ = 0;
                return e;
            }
        }
        return Event::None;
    }

    void reset() { *this = TapDetector{}; }

private:
    bool   was_pressed_{false};
    bool   pending_{false};       // taps counted, waiting for the window to close
    bool   void_{false};
    int    count_{0};
    double press_s_{0.0};
    double since_release_s_{0.0};
};

} // namespace kist
