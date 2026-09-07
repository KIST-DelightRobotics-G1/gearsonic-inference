#pragma once

#include "pico/pico_vr_body_pose.hpp"
#include "teleop/smpl_pose.hpp"

namespace kist {

// Per-arm running estimate of the operator's reach (tracked shoulder->wrist
// distance, metres) — how far "fully extended" is for this operator. Grows
// as they extend; no calibration gesture needed. Reset on engage.
struct ArmReachState {
    double max_reach[2] = {0.55, 0.55};  // [L, R], seeded at an adult default
};

// Re-solve the SmplPose arms from the TRACKED wrist positions instead of the
// tracked joint rotations. The headset's arm-joint orientations are IK
// estimates that under-represent extension (a straight operator arm comes
// out bent), while its wrist positions are exact — the upper-body flavor
// follows them well. So: keep the rotation-based FK for torso, legs, head
// and shoulders; for each arm take the tracked shoulder->wrist vector (same
// Unity->robot, root-relative mapping the 3-point path uses; the SMPL
// root-local frame is x-forward/y-left/z-up too), scale it by canonical /
// operator reach, and place the canonical two-link arm along it with the FK
// elbow as the swivel hint. Thumb tips follow the wrist rigidly.
void smpl_arms_from_tracked_wrists(SmplPose& pose, const PicoVRBodyPose& body,
                                   ArmReachState& reach);

} // namespace kist
