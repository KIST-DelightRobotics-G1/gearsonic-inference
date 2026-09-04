// Gate A for the smpl (full-body) encoder mode: the C++ SmplPose
// conversion (teleop/smpl_fk) must match gear_sonic's Python original
// numerically.
//
// Reads vectors produced by test/gen_smpl_vectors.py (which drives the
// UPSTREAM torch/scipy functions): one case per line, 250 numbers —
//   24 x [x y z qx qy qz qw]   operator body (input)
//   24 x [x y z]               smpl_joints_local (expected)
//    1 x [w x y z]             anchor quat (expected)
//    6 x wrist joint targets   q[23..28] (expected)
//
//   ./test_smpl_pose <vectors.txt> [joint_tol=5e-5] [quat_tol=1e-5] [wrist_tol=1e-4]
//
// Default tolerances allow for the reference running in float32 (as the
// upstream teleop stack does): joints land ~5e-7 m off, anchor ~2e-7, and
// the wrist Euler extraction amplifies that to ~2e-5 rad at large angles.
//
// Vectors from gen_smpl_vectors.py --double agree to ~2e-8 — pass
//   5e-8 5e-8 5e-8
// to check that. The 2e-8 floor is upstream's, not ours: its jit-scripted
// smpl_root_ytoz_up builds the Y-up->Z-up quaternion from a float32 pi/2
// (7e-8 rad off), which reaches the anchor directly and the joints through
// the root rest offset J0 (~0.35 m, not rotated by FK so it does not cancel)
// as a uniform ~2e-8 m shift. Both are below float32 encoder-input resolution.
//
// Exit 0 when every case is within tolerance; prints the max errors either way.

#include "teleop/smpl_fk.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace kist;

namespace {

// Quaternions are sign-ambiguous: distance to the closer of q and -q.
double quat_dist(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    double dp = 0.0, dm = 0.0;
    for (int i = 0; i < 4; ++i) {
        dp = std::max(dp, std::fabs(a[i] - b[i]));
        dm = std::max(dm, std::fabs(a[i] + b[i]));
    }
    return std::min(dp, dm);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <vectors.txt> [joint_tol] [quat_tol] [wrist_tol]\n", argv[0]);
        return 2;
    }
    const double joint_tol = argc > 2 ? std::atof(argv[2]) : 5e-5;
    const double quat_tol  = argc > 3 ? std::atof(argv[3]) : 1e-5;
    const double wrist_tol = argc > 4 ? std::atof(argv[4]) : 1e-4;

    std::ifstream in(argv[1]);
    if (!in) { std::fprintf(stderr, "cannot open %s\n", argv[1]); return 2; }

    constexpr int kPerCase = 24 * 7 + 24 * 3 + 4 + 6;  // 250
    int cases = 0, failed = 0;
    double max_j = 0.0, max_q = 0.0, max_w = 0.0;
    int worst_j_case = -1, worst_j_joint = -1;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::vector<double> v; v.reserve(kPerCase);
        double x;
        while (ss >> x) v.push_back(x);
        if (static_cast<int>(v.size()) != kPerCase) {
            std::fprintf(stderr, "case %d: expected %d numbers, got %zu\n", cases, kPerCase, v.size());
            return 2;
        }
        size_t k = 0;
        PicoVRBodyPose body{};
        for (auto& j : body.joints) for (auto& c : j) c = v[k++];
        SmplPose exp{};
        for (auto& j : exp.joints_local) for (auto& c : j) c = v[k++];
        for (auto& c : exp.anchor_quat) c = v[k++];
        for (auto& c : exp.wrist_joint_pos) c = v[k++];

        SmplPose got = smpl_fk(body);

        bool ok = true;
        for (int j = 0; j < 24; ++j)
            for (int c = 0; c < 3; ++c) {
                double e = std::fabs(got.joints_local[j][c] - exp.joints_local[j][c]);
                if (e > max_j) { max_j = e; worst_j_case = cases; worst_j_joint = j; }
                if (e > joint_tol) ok = false;
            }
        double eq = quat_dist(got.anchor_quat, exp.anchor_quat);
        max_q = std::max(max_q, eq);
        if (eq > quat_tol) ok = false;
        for (int i = 0; i < 6; ++i) {
            double e = std::fabs(got.wrist_joint_pos[i] - exp.wrist_joint_pos[i]);
            max_w = std::max(max_w, e);
            if (e > wrist_tol) ok = false;
        }
        if (!ok) {
            ++failed;
            if (failed <= 5) {
                std::printf("FAIL case %d: joint_err=%.3e quat_err=%.3e wrist_err=%.3e\n",
                            cases, max_j, eq, max_w);
                std::printf("  anchor got  [%.6f %.6f %.6f %.6f]\n", got.anchor_quat[0], got.anchor_quat[1], got.anchor_quat[2], got.anchor_quat[3]);
                std::printf("  anchor exp  [%.6f %.6f %.6f %.6f]\n", exp.anchor_quat[0], exp.anchor_quat[1], exp.anchor_quat[2], exp.anchor_quat[3]);
                std::printf("  wrist  got  [%.5f %.5f %.5f %.5f %.5f %.5f]\n", got.wrist_joint_pos[0], got.wrist_joint_pos[1], got.wrist_joint_pos[2], got.wrist_joint_pos[3], got.wrist_joint_pos[4], got.wrist_joint_pos[5]);
                std::printf("  wrist  exp  [%.5f %.5f %.5f %.5f %.5f %.5f]\n", exp.wrist_joint_pos[0], exp.wrist_joint_pos[1], exp.wrist_joint_pos[2], exp.wrist_joint_pos[3], exp.wrist_joint_pos[4], exp.wrist_joint_pos[5]);
            }
        }
        ++cases;
    }

    std::printf("test_smpl_pose: %d cases, %d failed\n", cases, failed);
    std::printf("  max joint err %.3e m (case %d, joint %d; tol %.1e)\n", max_j, worst_j_case, worst_j_joint, joint_tol);
    std::printf("  max anchor quat err %.3e (tol %.1e)\n", max_q, quat_tol);
    std::printf("  max wrist err %.3e rad (tol %.1e)\n", max_w, wrist_tol);
    if (cases == 0) { std::printf("  no cases read\n"); return 2; }
    return failed == 0 ? 0 : 1;
}
