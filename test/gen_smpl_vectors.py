#!/usr/bin/env python3
"""Generate Gate A vectors for test_smpl_pose from gear_sonic's ORIGINAL code.

Drives the upstream torch/scipy functions (compute_human_joints,
angle_axis_to_quaternion, smpl_root_ytoz_up, remove_smpl_base_rot,
decompose_rotation_aa, ...) with the glue copied verbatim from
gear_sonic/scripts/pico_manager_thread_server.py (compute_from_body_poses,
process_smpl_joints, the wrist block). One case per output line:
  24x[x y z qx qy qz qw]  24x[x y z]  [w x y z]  6 wrist targets   (250 numbers)

  python3 test/gen_smpl_vectors.py --out vectors.txt [--n 200] [--seed 0]
                                   [--gr00t ~/GR00T-WholeBodyControl]
Needs torch + scipy (a CPU torch is fine).
"""
import argparse, os, sys
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--n", type=int, default=200)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--gr00t", default=os.path.expanduser("~/GR00T-WholeBodyControl"))
ap.add_argument("--double", action="store_true",
                help="run the reference in float64 (upstream runs float32) to check exact agreement")
args = ap.parse_args()

# compute_human_joints loads its pkl by a repo-relative path.
sys.path.insert(0, args.gr00t)
os.chdir(args.gr00t)

import torch
from scipy.spatial.transform import Rotation as sRot
from scipy.spatial.transform import Rotation as R
from gear_sonic.trl.utils.rotation_conversion import decompose_rotation_aa
from gear_sonic.trl.utils.torch_transform import (
    angle_axis_to_quaternion, compute_human_joints, quat_apply, quat_inv,
    quaternion_to_angle_axis,
)
from gear_sonic.isaac_utils.rotations import remove_smpl_base_rot, smpl_root_ytoz_up
import gear_sonic.trl.utils.torch_transform as _tt

FDT = torch.float64 if args.double else torch.float32
if args.double:
    # compute_human_joints keeps the pkl skeleton (float32) in a module
    # global; cast it so the whole FK runs in float64.
    _tt.human_joints_info = torch.load("gear_sonic/data/human/human_joints_info.pkl", weights_only=False)
    _tt.human_joints_info["J"] = _tt.human_joints_info["J"].double()

# pico_manager parent_indices (verbatim, incl. its 23->22)
PARENTS = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 22, 23][:24]
device = "cpu"


# ── verbatim: pico_manager_thread_server.compute_from_body_poses ─────────────
def compute_from_body_poses(parent_indices, body_poses_np):
    positions = body_poses_np[:, :3]
    global_quats = body_poses_np[:, [3, 4, 5, 6]]          # xyzw for scipy (== scalar_first=True on [6,3,4,5])
    global_rots = sRot.from_quat(global_quats)
    global_rots = global_rots * sRot.from_euler("y", 180, degrees=True)
    local_rots = []
    for i in range(24):
        if parent_indices[i] == -1:
            local_rots.append(global_rots[i])
        else:
            local_rots.append(global_rots[parent_indices[i]].inv() * global_rots[i])
    pose_aa = np.array([rot.as_rotvec() for rot in local_rots])
    body_pose = torch.from_numpy(pose_aa[1:].flatten()).to(FDT).to(device).unsqueeze(0)
    global_orient = torch.from_numpy(pose_aa[0]).to(FDT).to(device).unsqueeze(0)
    transl = torch.from_numpy(positions[0]).to(FDT).to(device).unsqueeze(0)
    return process_smpl_joints(body_pose, global_orient, transl)


# ── verbatim: pico_manager_thread_server.process_smpl_joints ─────────────────
def process_smpl_joints(body_pose, global_orient, transl):
    global_orient_quat = angle_axis_to_quaternion(global_orient)
    global_orient_quat = smpl_root_ytoz_up(global_orient_quat)
    global_orient_new = quaternion_to_angle_axis(global_orient_quat)
    joints = compute_human_joints(body_pose=body_pose[..., :63], global_orient=global_orient_new)
    global_orient_quat = remove_smpl_base_rot(global_orient_quat, w_last=False)
    global_orient_quat_inv = quat_inv(global_orient_quat).unsqueeze(1).repeat(1, joints.shape[1], 1)
    smpl_joints_local = quat_apply(global_orient_quat_inv, joints)
    return {"smpl_pose": body_pose, "smpl_joints_local": smpl_joints_local,
            "global_orient_quat": global_orient_quat}


# ── verbatim: pico_manager_thread_server wrist block (lines ~1419-1477) ──────
def wrist_joint_pos(use_pose):
    joint_pos = np.zeros(29)
    body_pose = use_pose.reshape(-1, 21, 3)
    SMPL_L_ELBOW_IDX, SMPL_L_WRIST_IDX, SMPL_R_ELBOW_IDX, SMPL_R_WRIST_IDX = 17, 19, 18, 20
    G1_L_WRIST_ROLL_IDX, G1_L_WRIST_PITCH_IDX, G1_L_WRIST_YAW_IDX = 23, 25, 27
    G1_R_WRIST_ROLL_IDX, G1_R_WRIST_PITCH_IDX, G1_R_WRIST_YAW_IDX = 24, 26, 28
    smpl_l_elbow_aa = body_pose[:, SMPL_L_ELBOW_IDX]
    smpl_l_wrist_aa = body_pose[:, SMPL_L_WRIST_IDX]
    smpl_r_elbow_aa = body_pose[:, SMPL_R_ELBOW_IDX]
    smpl_r_wrist_aa = body_pose[:, SMPL_R_WRIST_IDX]
    g1_l_elbow_axis = np.array([0, 1, 0])
    _, g1_l_elbow_q_swing = decompose_rotation_aa(smpl_l_elbow_aa, g1_l_elbow_axis)
    g1_r_elbow_axis = np.array([0, 1, 0])
    _, g1_r_elbow_q_swing = decompose_rotation_aa(smpl_r_elbow_aa, g1_r_elbow_axis)
    l_elbow_swing_euler = R.from_quat(g1_l_elbow_q_swing[:, [1, 2, 3, 0]]).as_euler("XYZ", degrees=False)
    r_elbow_swing_euler = R.from_quat(g1_r_elbow_q_swing[:, [1, 2, 3, 0]]).as_euler("XYZ", degrees=False)
    l_wrist_euler = R.from_rotvec(smpl_l_wrist_aa).as_euler("XYZ", degrees=False)
    r_wrist_euler = R.from_rotvec(smpl_r_wrist_aa).as_euler("XYZ", degrees=False)
    g1_l_wrist_roll = l_elbow_swing_euler[:, 0] + l_wrist_euler[:, 0]
    g1_l_wrist_pitch = -l_wrist_euler[:, 1]
    g1_l_wrist_yaw = l_elbow_swing_euler[:, 2] + l_wrist_euler[:, 2]
    g1_r_wrist_roll = -(r_elbow_swing_euler[:, 0] + r_wrist_euler[:, 0])
    g1_r_wrist_pitch = -r_wrist_euler[:, 1]
    g1_r_wrist_yaw = r_elbow_swing_euler[:, 2] + r_wrist_euler[:, 2]
    joint_pos[G1_L_WRIST_ROLL_IDX] = g1_l_wrist_roll[0]
    joint_pos[G1_L_WRIST_PITCH_IDX] = -g1_l_wrist_pitch[0]
    joint_pos[G1_L_WRIST_YAW_IDX] = g1_l_wrist_yaw[0]
    joint_pos[G1_R_WRIST_ROLL_IDX] = g1_r_wrist_roll[0]
    joint_pos[G1_R_WRIST_PITCH_IDX] = g1_r_wrist_pitch[0]
    joint_pos[G1_R_WRIST_YAW_IDX] = g1_r_wrist_yaw[0]
    return joint_pos[23:29]


# ── case generation ──────────────────────────────────────────────────────────
rng = np.random.default_rng(args.seed)

def random_body(scale):
    """24 joints [x y z qx qy qz qw]: root free yaw + small tilt, limbs up to
    `scale` rad, elbows/wrists kept non-degenerate (decompose divides by |aa|)."""
    body = np.zeros((24, 7))
    body[:, :3] = rng.normal(0.0, 0.5, size=(24, 3))
    for i in range(24):
        if i == 0:
            rv = np.array([rng.normal(0, 0.15), rng.uniform(-np.pi, np.pi), rng.normal(0, 0.15)])
        else:
            rv = rng.normal(0.0, scale, size=3)
            if i in (18, 19, 20, 21) and np.linalg.norm(rv) < 0.05:
                rv = rv + 0.1
        body[i, 3:] = sRot.from_rotvec(rv).as_quat()       # xyzw
    return body

cases = []
# small / medium / large motion bands
for k in range(args.n):
    scale = [0.15, 0.5, 1.0][k % 3]
    cases.append(random_body(scale))

with open(args.out, "w") as f:
    f.write("# gen_smpl_vectors.py seed=%d n=%d — 24x7 body | 24x3 joints_local | anchor wxyz | 6 wrists\n"
            % (args.seed, args.n))
    bad = 0
    for body in cases:
        out = compute_from_body_poses(PARENTS, body)
        jl = out["smpl_joints_local"][0].detach().numpy().reshape(-1)     # 72
        anc = out["global_orient_quat"][0].detach().numpy().reshape(-1)   # 4 wxyz
        wr = wrist_joint_pos(out["smpl_pose"][0, :63].detach().numpy())    # 6
        row = np.concatenate([body.reshape(-1), jl, anc, wr])
        if not np.all(np.isfinite(row)):
            bad += 1
            continue
        f.write(" ".join("%.17g" % v for v in row) + "\n")
print("wrote %s: %d cases (%d skipped non-finite)" % (args.out, len(cases) - bad, bad))
