#pragma once

#include "pico/pico_vr_body_pose.hpp"
#include "teleop/smpl_pose.hpp"

namespace kist {

// SMPL forward kinematics on the operator's tracked body: one PICO 24-joint
// body pose (headset frame) -> SmplPose.
// Port of gear_sonic's pico_manager_thread_server.py path
// (compute_from_body_poses -> process_smpl_joints -> the wrist block):
// global joint rotations are re-expressed as parent-relative axis-angles,
// run through SMPL forward kinematics on the canonical skeleton (so operator
// body size drops out), and expressed in the root frame. Must stay
// numerically identical to the Python original — test_smpl_pose checks it
// against vectors made with the upstream functions.
SmplPose smpl_fk(const PicoVRBodyPose& body);

} // namespace kist
