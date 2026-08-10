#pragma once

#include <atomic>

namespace kist {

// Who is driving the robot: first-come, keep-until-done arbitration between
// the external VLA token stream and VR teleop.
//
// Policy (deliberately simple — no preemption, no priority):
//   - Whichever side claims first OWNS the robot; the other side's inputs
//     are ignored for as long as the owner holds.
//   - Teleop claims on calibration and releases when calibration drops.
//   - VLA claims on its first fresh token and NEVER auto-releases: if the
//     stream dies the controller latches damping (a stale planner motion or
//     a mid-task teleop takeover would be an uncommanded movement) — VLA
//     ownership ends only with a process restart.
//   - The VR grip e-stop is NOT part of this arbitration: it overrides
//     everything, always, in both control loops.
//
// Header-only and lock-free so any module (control, hand writer) can consult
// it without new link dependencies.
class RobotOwnership {
public:
    enum class Owner { kNone, kVla, kTeleop };

    static RobotOwnership& instance() {
        static RobotOwnership inst;
        return inst;
    }

    // Attempt to claim for `who`. Returns true when `who` is the owner after
    // the call (freshly claimed, or already held by `who`).
    bool try_claim(Owner who) {
        Owner expected = Owner::kNone;
        if (owner_.compare_exchange_strong(expected, who))
            return true;
        return expected == who;
    }

    // Release only succeeds for the current owner; anyone else is a no-op.
    void release(Owner who) {
        Owner expected = who;
        owner_.compare_exchange_strong(expected, Owner::kNone);
    }

    Owner owner() const { return owner_.load(); }

private:
    RobotOwnership() = default;
    std::atomic<Owner> owner_{Owner::kNone};
};

} // namespace kist
