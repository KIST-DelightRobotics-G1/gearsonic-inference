#pragma once

#include "common/data_buffer.hpp"
#include "motion/movement_state.hpp"
#include "motion/nav_command.hpp"
#include <atomic>
#include <thread>

namespace kist {

class InputHandler {
public:
    // ── lifecycle ──────────────────────────────────────────────
    static InputHandler& instance();
    void                 start();
    void                 stop();

    // ── data buffers ────────────────────────────────────────────
    // Output: the single source of planner movement commands.
    DataBuffer<MovementState> movement_buf;

    // Input: external navigation velocity (written by the embedding
    // process, e.g. the umbrella repo's path-follower wiring).
    // Arbitration is momentary: any joystick deflection wins instantly,
    // releasing the stick hands control back to nav on the next tick;
    // a stale/empty buffer falls back to manual. Nav is also ignored
    // while the VR link is down — the controller is the e-stop, so
    // nothing may drive the robot without it.
    DataBuffer<NavCommand> nav_buf;

    // Selected locomotion mode — what the stick will trigger. The
    // published MovementState stays IDLE while the stick is in the
    // deadzone (unless a fresh NavCommand is driving), so displays
    // should show this instead.
    int mode() const { return mode_index_; }

    // Drop back to IDLE so walking requires an explicit re-selection —
    // same semantics as the VR-link-loss safe stop. Called by the control
    // loop when the origin is re-entered after a VLA session.
    void disarm() { mode_index_ = 0; }

    // Emergency stop: latched once A+B+X+Y are all held for 1s.
    // One-way — never clears until the process restarts. Deliberately an
    // atomic and not a DataBuffer: a stop request must not be clearable
    // or go stale.
    bool estop() const { return estop_; }

private:
    InputHandler() = default;

    void loop();

    // "Held alone for N ticks" edge detector — decouples the physical
    // press (which cannot happen simultaneously across buttons at tick
    // resolution) from the logical single-button gesture. A concurrent
    // press on any other face button resets the counter and voids the
    // gesture, letting A+B+X+Y (or any 2-button pair) be safely detected
    // as a combo instead of firing whichever button ticked first.
    struct HoldTrigger {
        int  ticks{0};
        bool fired{false};
        // Returns true exactly once when active_alone has held for `threshold`
        // ticks. Reset by any release (or any tick with combo=true).
        bool tick(bool active_alone, int threshold) {
            if (!active_alone) { ticks = 0; fired = false; return false; }
            if (fired) return false;
            if (++ticks >= threshold) { fired = true; return true; }
            return false;
        }
    };

    double facing_angle_{0.0};
    double height_{-1.0};  // valid only in crouch modes; -1 = mode default
    // Arming a locomotion mode is a deliberate operator act: start in IDLE,
    // and drop back to IDLE whenever the VR link is lost (gear_sonic default).
    std::atomic<int> mode_index_{0};  // IDLE

    HoldTrigger a_press_, x_press_, y_press_;
    int    estop_hold_ticks_{0};
    std::atomic<bool> estop_{false};

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
