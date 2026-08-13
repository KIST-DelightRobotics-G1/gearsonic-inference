#include "motion/input_handler.hpp"
#include "control/control_arbiter.hpp"
#include "pico/pico_vr_reader.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>

namespace kist {

// Mode navigation walks the LocomotionMode values linearly (gear_sonic
// remote scheme, clamped at both ends). Going up past WALK — RUN and the
// advanced modes (squat, crawl, boxing, ...) — requires holding a trigger;
// going down (toward IDLE — always the safer direction) is never gated.
static constexpr int    kBasicModeMax = static_cast<int>(LocomotionMode::WALK);         // 2
static constexpr int    kFullModeMax  = static_cast<int>(LocomotionMode::INJURED_WALK); // 19 (gear_sonic cap)
static constexpr double kTriggerHeld  = 0.5;
// Body height is meaningful only in the crouch modes (squat/kneel, 4..6);
// standing modes force -1 (mode default), matching the gear_sonic gamepad.
// Adjusted by trigger+B (up) / trigger+A (down) while crouched.
static constexpr int    kCrouchFirst  = static_cast<int>(LocomotionMode::IDEL_SQUAT); // 4
static constexpr int    kCrouchLast   = static_cast<int>(LocomotionMode::IDEL_KNEEL); // 6
static constexpr double kHeightSeed   = 0.8;   // on entering a crouch mode (gear_sonic)
static constexpr double kHeightMin    = 0.1;   // gear_sonic clamp
static constexpr double kHeightMax    = 0.8;
static constexpr double kHeightRate   = 0.3;   // m/s while held
// E-stop gesture: all four face buttons held for a full second. Buttons
// (not the grip axes) so hand control — grip=thumb, trigger=index+middle
// — stays orthogonal; four-buttons+hold rules out reflex/accident.
// Single-button actions defer 30 ms to distinguish deliberate presses
// from the first-arriving buttons of a combo (rising edges never land on
// the same tick at 500Hz, so a naive edge detector would fire whichever
// button arrived first — resetting the mode or toggling teleop on the
// way into an e-stop). Any two face buttons pressed at once cancels all
// single-button actions until the buttons release.
static constexpr int kEstopHoldTicks   = 500;  // 1s at 500Hz
static constexpr int kSingleHoldTicks  = 15;   // 30ms at 500Hz — combo grace
static constexpr double kDeadZone   = 0.15;   // gear_sonic JOYSTICK_DEADZONE
static constexpr double kFacingRate = 1.5;    // rad/s at full stick (gear_sonic yaw_gain)
static constexpr double kLoopDt     = 0.002;  // 500Hz
// External nav commands: fresh within this window (producer ~20Hz), and
// |v| below the stop threshold means "arrived — stand still". Speeds up
// to the slow-walk band map onto SLOW_WALK; anything faster uses WALK's
// mode-default speed.
static constexpr double kNavStaleMs   = 300.0;
static constexpr double kNavStopSpeed = 0.05;  // m/s
static constexpr double kNavSlowMax   = 0.6;   // SLOW_WALK speed ceiling

InputHandler& InputHandler::instance() {
    static InputHandler inst;
    return inst;
}

void InputHandler::start() {
    stop_ = false;
    loop_thread_ = std::thread(&InputHandler::loop, this);
    std::cout << "[InputHandler] started\n";
}

void InputHandler::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
}

void InputHandler::loop() {
    using namespace std::chrono_literals;
    using clock = std::chrono::steady_clock;

    constexpr auto period = std::chrono::microseconds(2000); // 500Hz

    while (!stop_) {
        auto t0 = clock::now();

        auto ctrl = PicoVRReader::instance().ctrl_buf.GetData();
        if (ctrl) {
            double lx = ctrl->left_axis[0];
            double ly = ctrl->left_axis[1];
            double rx = ctrl->right_axis[0];

            // ── combo detection ────────────────────────────────────
            // Face buttons only. Trigger/grip are analog and drive the
            // Dex3-1 hand — never gate them by combo state.
            int n_face = (int)ctrl->btn_a + (int)ctrl->btn_b +
                         (int)ctrl->btn_x + (int)ctrl->btn_y;
            bool combo = n_face >= 2;

            // ── emergency stop (latched) ──────────────────────────
            // All four face buttons held together for 1s. Combo suppression
            // below prevents any per-button action from firing on the way
            // in.
            if (n_face == 4) {
                if (++estop_hold_ticks_ >= kEstopHoldTicks && !estop_) {
                    estop_ = true;
                    std::cerr << "[InputHandler] EMERGENCY STOP (A+B+X+Y held)\n";
                }
            } else {
                estop_hold_ticks_ = 0;
            }

            // ── mode selection (single-button, 30ms grace) ────────
            bool trigger_held = ctrl->left_trigger > kTriggerHeld ||
                                ctrl->right_trigger > kTriggerHeld;

            // Every single-button gesture defers by kSingleHoldTicks: any
            // concurrent face button in that window resets the counter,
            // voiding the gesture. Combos never trigger single-button
            // effects.
            bool a_alone = ctrl->btn_a && !combo && !trigger_held;
            bool y_alone = ctrl->btn_y && !combo;
            bool x_alone = ctrl->btn_x && !combo;

            if (a_press_.tick(a_alone, kSingleHoldTicks))
                mode_index_ = static_cast<int>(LocomotionMode::IDLE);  // escape from anywhere
            if (y_press_.tick(y_alone, kSingleHoldTicks)) {
                int limit = trigger_held ? kFullModeMax : kBasicModeMax;
                if (mode_index_ < limit)
                    ++mode_index_;
            }
            if (x_press_.tick(x_alone, kSingleHoldTicks) && mode_index_ > 0)
                --mode_index_;

            // ── body height (crouch modes only) ──────────────────
            bool crouched = mode_index_ >= kCrouchFirst && mode_index_ <= kCrouchLast;
            if (crouched) {
                if (height_ < 0.0)
                    height_ = kHeightSeed;  // just entered the crouch band
                // Height rides trigger+B (up) / trigger+A (down). Gate
                // by !combo so an incoming e-stop doesn't jog height on
                // its way in.
                if (trigger_held && !combo) {
                    if (ctrl->btn_b) height_ += kHeightRate * kLoopDt;  // B = up
                    if (ctrl->btn_a) height_ -= kHeightRate * kLoopDt;
                    height_ = std::clamp(height_, kHeightMin, kHeightMax);
                }
            } else {
                height_ = -1.0;
            }

            // ── facing angle ──────────────────────────────────────
            if (std::abs(rx) > kDeadZone)
                facing_angle_ -= kFacingRate * rx * kLoopDt;

            // ── build MovementState (gear_sonic pico manager scheme:
            //    stick magnitude drives speed, direction stays continuous) ──
            std::array<double, 3> final_face = {std::cos(facing_angle_), std::sin(facing_angle_), 0.0};

            int                   final_mode;
            double                final_speed;
            double                final_height = height_;
            std::array<double, 3> final_move{0.0, 0.0, 0.0};

            double raw_mag = std::hypot(lx, ly);
            if (raw_mag < kDeadZone) {
                final_mode  = static_cast<int>(LocomotionMode::IDLE);
                final_speed = -1.0;

                // ── nav arbitration: stick neutral -> a fresh external
                //    command drives instead (momentary priority). Origin
                //    mode only: during teleop the operator's stick is the
                //    sole locomotion source, and during VLA/recovery the
                //    planner output is unconsumed anyway ─────
                auto nav = nav_buf.GetDataWithTime();
                if (ControlArbiter::instance().mode() == ControlArbiter::Mode::kNormal &&
                    nav.HasData() && nav.GetAgeMs() < kNavStaleMs) {
                    double v = std::hypot(nav.data->vx, nav.data->vy);
                    final_height = -1.0;  // nav modes are non-crouch

                    // yaw rate integrates into facing, joystick-style
                    facing_angle_ += nav.data->vyaw * kLoopDt;
                    final_face = {std::cos(facing_angle_), std::sin(facing_angle_), 0.0};

                    if (v >= kNavStopSpeed) {
                        if (v <= kNavSlowMax) {
                            final_mode  = static_cast<int>(LocomotionMode::SLOW_WALK);
                            final_speed = std::max(v, 0.1);
                        } else {
                            final_mode  = static_cast<int>(LocomotionMode::WALK);
                            final_speed = -1.0;  // mode default
                        }
                        // body-frame (vx fwd, vy left) -> world via facing
                        double fwd_c = nav.data->vx / v;
                        double left_c = nav.data->vy / v;
                        final_move = {
                            -final_face[1] * left_c + final_face[0] * fwd_c,
                             final_face[0] * left_c + final_face[1] * fwd_c,
                             0.0
                        };
                    }
                    // v < stop threshold: keep IDLE — nav says "stand still"
                }
            } else {
                double mag = std::min((raw_mag - kDeadZone) / (1.0 - kDeadZone), 1.0);
                final_mode = mode_index_;

                switch (static_cast<LocomotionMode>(final_mode)) {
                    case LocomotionMode::SLOW_WALK: final_speed = 0.1 + 0.5 * mag; break;
                    case LocomotionMode::WALK:      final_speed = -1.0;            break;
                    case LocomotionMode::RUN:       final_speed = 1.5 + 3.0 * mag; break;
                    default:                        final_speed = mag;             break;
                }

                // local stick vector, deadzone-rescaled, rotated into the
                // facing frame (gear_sonic: [[-fy, fx], [fx, fy]] @ [-lx, ly])
                double scale = mag / raw_mag;
                double ml0 = -lx * scale;
                double ml1 =  ly * scale;
                final_move = {
                    -final_face[1] * ml0 + final_face[0] * ml1,
                     final_face[0] * ml0 + final_face[1] * ml1,
                     0.0
                };
            }

            movement_buf.SetData(MovementState(final_mode, final_move, final_face, final_speed, final_height));
        } else {
            // VR link lost (stale watchdog cleared ctrl_buf) → safe stop:
            // IDLE with mode-default speed/height. Keep the current facing so
            // the robot doesn't turn toward yaw 0 while idling. The selected
            // mode stays armed — when the link recovers, the stick drives
            // again without re-selecting (the 350ms stale window already
            // absorbs ordinary WiFi hiccups; this covers the longer ones).
            std::array<double, 3> face{std::cos(facing_angle_), std::sin(facing_angle_), 0.0};
            movement_buf.SetData(MovementState(
                static_cast<int>(LocomotionMode::IDLE), {0.0, 0.0, 0.0}, face, -1.0, -1.0));
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

} // namespace kist
