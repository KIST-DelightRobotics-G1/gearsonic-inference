#include "teleop/smpl_arm_reach.hpp"
#include "teleop/smpl_skeleton.hpp"
#include "common/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace kist {

namespace {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

// Headset -> robot frame, as teleop_tracker.cpp's extract_raw (must match):
// positions [x,y,z] -> [-x, z, y]; orientations by conjugation with the same
// proper rotation; the SMPL root then carries a -90° yaw offset.
const Quat kUnityToRobot = {0.0, 0.0, std::sqrt(0.5), std::sqrt(0.5)};
const Quat kRootYawOffset = quat_from_angle_axis(-M_PI / 2.0, {0.0, 0.0, 1.0});

Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
Vec3 add(const Vec3& a, const Vec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
Vec3 scale(const Vec3& a, double s)    { return {a[0]*s, a[1]*s, a[2]*s}; }
double dot(const Vec3& a, const Vec3& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]};
}

// Tracked joint position in the operator's root (pelvis) frame, robot axes.
Vec3 tracked_root_relative(const PicoVRBodyPose& body, int joint) {
    auto pos_robot = [&](int k) -> Vec3 {
        const auto& j = body.joints[k];
        return {-j[0], j[2], j[1]};
    };
    const auto& r = body.joints[0];
    Quat q_root = {r[6], r[3], r[4], r[5]};
    q_root = quat_mul(kUnityToRobot, quat_mul(q_root, quat_conjugate(kUnityToRobot)));
    q_root = quat_mul(q_root, kRootYawOffset);
    return quat_rotate(quat_conjugate(q_root), sub(pos_robot(joint), pos_robot(0)));
}

// Canonical segment lengths from the rest skeleton.
double seg_len(int a, int b) {
    return norm(sub(kSmplXRestJoints[a], kSmplXRestJoints[b]));
}

// One arm: SMPL-24 shoulder s, elbow e, wrist w; joints_local slot of the
// thumb tip t; tracked wrist keypoint tw (the stream's 22/23, as the 3-point
// path uses). side 0 = L, 1 = R.
void solve_arm(SmplPose& pose, const PicoVRBodyPose& body, ArmReachState& reach,
               int side, int s, int e, int w, int t, int tw) {
    const double a = seg_len(s, e), b = seg_len(e, w);

    Vec3 v = sub(tracked_root_relative(body, tw), tracked_root_relative(body, s));
    double len = norm(v);
    if (len < 1e-3)
        return;  // degenerate tracking — keep the FK arm

    double& max_reach = reach.max_reach[side];
    max_reach = std::max(max_reach, len);
    double target = std::min(len / max_reach, 1.0) * (a + b);
    Vec3 u = scale(v, 1.0 / len);

    const Vec3 S = pose.joints_local[s];
    const Vec3 E_fk = pose.joints_local[e];
    const Vec3 W_fk = pose.joints_local[w];
    const Vec3 thumb_off = sub(pose.joints_local[t], W_fk);

    // Two-link IK along u: elbow at distance `along` from the shoulder on the
    // shoulder->wrist line, `r` off it, swivelled toward the FK elbow.
    double d = std::clamp(target, std::fabs(a - b) + 1e-6, a + b - 1e-6);
    double along = (a*a - b*b + d*d) / (2.0 * d);
    double r = std::sqrt(std::max(a*a - along*along, 0.0));

    Vec3 h = sub(E_fk, S);
    Vec3 h_perp = sub(h, scale(u, dot(h, u)));
    if (norm(h_perp) < 1e-4) {
        // FK elbow on the line: default swivel = down, else back
        h_perp = sub(Vec3{0.0, 0.0, -1.0}, scale(u, dot(Vec3{0.0, 0.0, -1.0}, u)));
        if (norm(h_perp) < 1e-4)
            h_perp = cross(u, {0.0, 1.0, 0.0});
    }
    h_perp = scale(h_perp, 1.0 / norm(h_perp));

    Vec3 E = add(S, add(scale(u, along), scale(h_perp, r)));
    Vec3 W = add(S, scale(u, d));

    pose.joints_local[e] = E;
    pose.joints_local[w] = W;
    pose.joints_local[t] = add(W, thumb_off);
}

} // namespace

void smpl_arms_from_tracked_wrists(SmplPose& pose, const PicoVRBodyPose& body,
                                   ArmReachState& reach) {
    // SMPL-24 chain: shoulders 16/17, elbows 18/19, wrists 20/21; joints_local
    // 22/23 are the thumb tips (SMPL-X 39/54); tracked wrists are stream 22/23.
    solve_arm(pose, body, reach, 0, 16, 18, 20, 22, 22);
    solve_arm(pose, body, reach, 1, 17, 19, 21, 23, 23);
}

} // namespace kist
