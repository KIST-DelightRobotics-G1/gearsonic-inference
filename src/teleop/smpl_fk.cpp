#include "teleop/smpl_fk.hpp"
#include "teleop/smpl_skeleton.hpp"
#include "common/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace kist {

namespace {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // wxyz
using Mat3 = std::array<std::array<double, 3>, 3>;

constexpr double kPi = 3.14159265358979323846;

// ─── rotation helpers not in math_utils ───────────────────────────────────────

// Axis-angle vector -> quaternion (scipy Rotation.from_rotvec).
Quat quat_from_rotvec(const Vec3& v) {
    double angle = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (angle < 1e-12)
        return {1.0, 0.0, 0.0, 0.0};
    return quat_from_angle_axis(angle, v);
}

// Quaternion -> axis-angle vector with angle in [0, pi]
// (scipy Rotation.as_rotvec canonical form).
Vec3 rotvec_from_quat(const Quat& q_in) {
    Quat q = quat_unit(q_in);
    if (q[0] < 0.0)
        for (auto& c : q) c = -c;
    double vn = std::sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (vn < 1e-12)
        return {0.0, 0.0, 0.0};
    double angle = 2.0 * std::atan2(vn, q[0]);
    return {q[1] / vn * angle, q[2] / vn * angle, q[3] / vn * angle};
}

// Intrinsic X-Y-Z Euler angles (scipy as_euler("XYZ")) from a rotation
// matrix M = Rx(a) * Ry(b) * Rz(c).
Vec3 euler_xyz_intrinsic(const Mat3& m) {
    // Pitch via atan2 (not asin) — well-conditioned up to the gimbal lock,
    // matching scipy's extraction to ~1e-12 instead of ~1e-8 at large angles.
    double cb = std::sqrt(m[0][0]*m[0][0] + m[0][1]*m[0][1]);
    return {std::atan2(-m[1][2], m[2][2]),
            std::atan2(m[0][2], cb),
            std::atan2(-m[0][1], m[0][0])};
}

Vec3 mat_apply(const Mat3& m, const Vec3& v) {
    return {m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
            m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
            m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]};
}

Mat3 mat_mul(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r[i][j] = a[i][0]*b[0][j] + a[i][1]*b[1][j] + a[i][2]*b[2][j];
    return r;
}

// gear_sonic decompose_rotation_aa: split an axis-angle rotation into the
// twist about `axis` and the remaining swing (q = twist * swing).
void swing_twist(const Vec3& rotvec, const Vec3& axis, Quat& twist, Quat& swing) {
    Quat q = quat_from_rotvec(rotvec);
    double d = q[1]*axis[0] + q[2]*axis[1] + q[3]*axis[2];
    twist = quat_unit({q[0], d*axis[0], d*axis[1], d*axis[2]});
    swing = quat_mul(quat_conjugate(twist), q);
}


// Rotation matrix from an axis-angle vector, reproducing kornia's
// angle_axis_to_rotation_matrix as gear_sonic's SMPL FK runs it — including
// its two numerical quirks, so the skeleton comes out bit-for-bit like the
// data the tokenizer was trained on: the axis is normalised by (theta + eps)
// rather than theta (a ~1e-6/theta relative shortening, worth ~4e-6 m at the
// hands), and rotations with theta^2 <= eps use the first-order I + [v]x.
// Only the FK uses this; everything else takes the exact conversions.
Mat3 rotmat_from_rotvec_kornia(const Vec3& v) {
    constexpr double eps = 1e-6;
    double theta2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    if (theta2 <= eps) {
        return {{{1.0, -v[2], v[1]},
                 {v[2], 1.0, -v[0]},
                 {-v[1], v[0], 1.0}}};
    }
    double theta = std::sqrt(std::max(theta2, eps));
    double wx = v[0] / (theta + eps), wy = v[1] / (theta + eps), wz = v[2] / (theta + eps);
    double c = std::cos(theta), s = std::sin(theta), k = 1.0 - c;
    return {{{c + wx*wx*k,      wx*wy*k - wz*s,  wy*s + wx*wz*k},
             {wz*s + wx*wy*k,   c + wy*wy*k,     -wx*s + wy*wz*k},
             {-wy*s + wx*wz*k,  wx*s + wy*wz*k,  c + wz*wz*k}}};
}

// ─── SMPL forward kinematics (gear_sonic compute_human_joints) ────────────────

// pose[i]: axis-angle of SMPL-X joint i (55). Returns the 55 posed joint
// positions; rest offsets/tree from smpl_skeleton.hpp.
std::array<Vec3, kSmplXNumJoints> smpl_fk_chain(const std::array<Vec3, kSmplXNumJoints>& pose) {
    std::array<Mat3, kSmplXNumJoints> rot;
    std::array<Vec3, kSmplXNumJoints> pos;
    for (int i = 0; i < kSmplXNumJoints; ++i) {
        Mat3 r = rotmat_from_rotvec_kornia(pose[i]);
        int p = kSmplXParents[i];
        Vec3 rel = kSmplXRestJoints[i];
        if (p >= 0)
            for (int k = 0; k < 3; ++k) rel[k] -= kSmplXRestJoints[p][k];
        if (p < 0) {
            rot[i] = r;
            pos[i] = rel;
        } else {
            rot[i] = mat_mul(rot[p], r);
            Vec3 t = mat_apply(rot[p], rel);
            for (int k = 0; k < 3; ++k) pos[i][k] = pos[p][k] + t[k];
        }
    }
    return pos;
}

// SMPL's default rest orientation, conjugated out of the root so the
// reference reads as a neutral standing pose (gear_sonic remove_smpl_base_rot).
const Quat kSmplBaseRot = {0.5, 0.5, 0.5, 0.5};

} // namespace

SmplPose smpl_fk(const PicoVRBodyPose& body) {
    // ── compute_from_body_poses: global -> parent-relative axis-angles ──
    // Stream quats are xyzw; the Y-180 right-multiply is the headset->SMPL
    // frame correction applied to every joint.
    const Quat y180 = quat_from_angle_axis(kPi, {0.0, 1.0, 0.0});
    std::array<Quat, kSmplNumJoints> global_rot;
    for (int i = 0; i < kSmplNumJoints; ++i) {
        const auto& j = body.joints[i];
        global_rot[i] = quat_mul(quat_unit({j[6], j[3], j[4], j[5]}), y180);
    }
    std::array<Vec3, kSmplNumJoints> pose_aa;
    for (int i = 0; i < kSmplNumJoints; ++i) {
        int p = kSmplStreamParents[i];
        Quat local = (p < 0) ? global_rot[i]
                             : quat_mul(quat_conjugate(global_rot[p]), global_rot[i]);
        pose_aa[i] = rotvec_from_quat(local);
    }

    // ── process_smpl_joints ──
    // Root: SMPL is Y-up, the robot Z-up — a +90° X rotation on the left.
    Quat q_root = quat_mul(quat_from_angle_axis(kPi / 2.0, {1.0, 0.0, 0.0}),
                           quat_from_rotvec(pose_aa[0]));

    // FK on the 55-joint tree: root + body joints 1..21 posed, hands/face
    // (22..54) zero — the thumb tips then follow the wrists rigidly.
    std::array<Vec3, kSmplXNumJoints> full_pose{};
    full_pose[0] = rotvec_from_quat(q_root);
    for (int i = 1; i <= 21; ++i) full_pose[i] = pose_aa[i];
    auto joints55 = smpl_fk_chain(full_pose);

    // Remove SMPL's rest rotation from the root, then express the 24
    // output joints in that root frame.
    q_root = quat_mul(q_root, quat_conjugate(kSmplBaseRot));
    Quat q_root_inv = quat_conjugate(q_root);

    SmplPose out{};
    for (int k = 0; k < kSmplNumJoints; ++k)
        out.joints_local[k] = quat_rotate(q_root_inv, joints55[kSmplOutputJoints[k]]);
    out.anchor_quat = q_root;

    // ── wrist block: G1 wrist joints from SMPL elbow (18/19) + wrist (20/21) ──
    // Elbow roll/yaw (the swing left after removing its Y twist) is folded
    // into the wrist; wrist pitch is SMPL's own.
    const Vec3 y_axis = {0.0, 1.0, 0.0};
    Quat tw, l_sw, r_sw;
    swing_twist(pose_aa[18], y_axis, tw, l_sw);
    swing_twist(pose_aa[19], y_axis, tw, r_sw);
    Vec3 l_elbow = euler_xyz_intrinsic(quat_to_rotation_matrix(l_sw));
    Vec3 r_elbow = euler_xyz_intrinsic(quat_to_rotation_matrix(r_sw));
    Vec3 l_wrist = euler_xyz_intrinsic(quat_to_rotation_matrix(quat_from_rotvec(pose_aa[20])));
    Vec3 r_wrist = euler_xyz_intrinsic(quat_to_rotation_matrix(quat_from_rotvec(pose_aa[21])));

    double l_roll  =  l_elbow[0] + l_wrist[0];
    double l_pitch = -l_wrist[1];
    double l_yaw   =  l_elbow[2] + l_wrist[2];
    double r_roll  = -(r_elbow[0] + r_wrist[0]);
    double r_pitch = -r_wrist[1];
    double r_yaw   =  r_elbow[2] + r_wrist[2];

    // q[23..28]: L roll, R roll, L pitch, R pitch, L yaw, R yaw. The left
    // pitch is negated once more on assignment, as in the original.
    out.wrist_joint_pos = {l_roll, r_roll, -l_pitch, r_pitch, l_yaw, r_yaw};
    return out;
}

} // namespace kist
