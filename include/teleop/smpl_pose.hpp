#pragma once

#include <array>

namespace kist {

// One frame of the operator's body as the encoder's smpl (full-body) mode
// consumes it — re-expressed on the canonical SMPL skeleton. Produced by
// teleop/smpl_fk from a PicoVRBodyPose; TeleopTracker stacks
// these into the SmplPoseWindow the encoder reads.
struct SmplPose {
    // 24 joint positions in the root (pelvis) frame — root rotation removed,
    // translation kept (gear_sonic smpl_joints_local). Metres.
    std::array<std::array<double, 3>, 24> joints_local;
    // Root orientation, wxyz: Z-up, SMPL rest rotation removed. Feeds the
    // smpl anchor-orientation observation and the heading alignment.
    std::array<double, 4> anchor_quat;
    // G1 wrist joint targets q[23..28] — L roll, R roll, L pitch, R pitch,
    // L yaw, R yaw (IsaacLab interleaved order), radians.
    std::array<double, 6> wrist_joint_pos;
};

} // namespace kist
