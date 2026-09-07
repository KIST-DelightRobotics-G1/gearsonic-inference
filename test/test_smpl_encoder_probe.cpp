// Gate B for the smpl (full-body) encoder mode, offline and deterministic.
//
// Ten SmplPose frames (the expected columns of a gen_smpl_vectors.py file)
// form an SmplPoseWindow; TokenEncoder::step runs it in smpl mode with a
// fixed robot base orientation. Writes the assembled observation and the
// token so gen_smpl_encoder_ref.py can compare them slot by slot against an
// independent Python assembly + onnxruntime, then perturbs input regions to
// show which slots the smpl path actually reads (the g1/teleop blocks must
// be dead in mode 2, the three smpl blocks live). Optional — hardware is the
// real check; this localises a wrong slot without the robot.
//
//   ./build/test_smpl_encoder_probe [encoder.onnx=models/model_encoder.onnx]
//                                   [vectors=test/data/smpl_vectors.txt] [out_dir=build]
//     -> <out_dir>/obs_kist.txt (1751 floats), <out_dir>/token_kist.txt (64)


#include "common/math_utils.hpp"
#include "control/obs_dict_model.hpp"
#include "control/token_encoder.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace kist;

namespace {

// Deterministic robot base: yaw 0.3 with a small roll/pitch, so the
// heading-only anchor (a lean must NOT rotate the reference) is exercised.
std::array<double, 4> probe_base_quat() {
    auto qz = quat_from_angle_axis(0.30, {0.0, 0.0, 1.0});
    auto qx = quat_from_angle_axis(0.05, {1.0, 0.0, 0.0});
    auto qy = quat_from_angle_axis(-0.04, {0.0, 1.0, 0.0});
    return quat_mul(qz, quat_mul(qx, qy));
}

bool load_window(const std::string& path, SmplPoseWindow& w) {
    std::ifstream in(path);
    if (!in) return false;
    constexpr int kPerCase = 24 * 7 + 24 * 3 + 4 + 6;
    std::string line;
    int f = 0;
    while (f < SmplPoseWindow::kFrames && std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::vector<double> v; v.reserve(kPerCase);
        double x;
        while (ss >> x) v.push_back(x);
        if (static_cast<int>(v.size()) != kPerCase) return false;
        size_t k = 24 * 7;  // skip the input body; take the expected outputs
        SmplPose& p = w.frames[f++];
        for (auto& j : p.joints_local) for (auto& c : j) c = v[k++];
        for (auto& c : p.anchor_quat) c = v[k++];
        for (auto& c : p.wrist_joint_pos) c = v[k++];
    }
    return f == SmplPoseWindow::kFrames;
}

void write_floats(const std::string& path, const float* v, size_t n) {
    std::ofstream out(path);
    for (size_t i = 0; i < n; ++i) out << (i ? " " : "") << std::scientific << v[i];
    out << "\n";
}

double max_abs_diff(const float* a, const float* b, size_t n) {
    double m = 0.0;
    for (size_t i = 0; i < n; ++i) m = std::max(m, static_cast<double>(std::fabs(a[i] - b[i])));
    return m;
}

} // namespace

int main(int argc, char** argv) {
    // Defaults assume the repo root as cwd (as inside the deploy container).
    const std::string onnx    = argc > 1 ? argv[1] : "models/model_encoder.onnx";
    const std::string vectors = argc > 2 ? argv[2] : "test/data/smpl_vectors.txt";
    const std::string out_dir = argc > 3 ? argv[3] : "build";

    SmplPoseWindow window;
    if (!load_window(vectors, window)) {
        std::fprintf(stderr, "could not read 10 frames from %s\n", vectors.c_str());
        return 2;
    }

    TokenEncoder encoder;
    if (!encoder.init(onnx)) return 1;

    // Fixed base for both the heading init and the anchor observation.
    StateLogger logger;
    for (int k = 0; k < 10; ++k) {
        StateLogger::Entry e;
        e.base_quat = probe_base_quat();
        logger.Log(e);
    }
    // smpl mode ignores the planner motion; one frame just satisfies the
    // non-empty guard.
    MotionSequence50Hz motion;
    motion.resize(1);
    motion.frames[0].quaternion = {1.0, 0.0, 0.0, 0.0};

    // ── smpl mode: baseline ──
    TokenEncoder::Token token;
    if (!encoder.step(motion, 0, /*playing=*/true, logger, nullptr, &window, token)) return 1;
    std::vector<float> obs(encoder.last_obs(), encoder.last_obs() + TokenEncoder::kInputDim);
    write_floats(out_dir + "/obs_kist.txt", obs.data(), obs.size());
    write_floats(out_dir + "/token_kist.txt", token.data(), token.size());
    std::printf("smpl mode: obs[0]=%.0f  token[0..3] = %.6f %.6f %.6f %.6f\n",
                obs[0], token[0], token[1], token[2], token[3]);

    // ── perturbation scan on the assembled observation ──
    // Same graph, direct input access: bump a region by +0.1 and see whether
    // the token moves. Expect: smpl blocks live, g1/teleop blocks dead.
    ObsDictModel probe;
    if (!probe.Initialize(onnx, "encoded_tokens", TokenEncoder::kInputDim, TokenEncoder::kTokenDim))
        return 1;
    struct Region { const char* name; size_t lo, hi; bool expect_live; };
    const Region regions[] = {
        {"g1 joints  [4:294]",        4,    294,  false},
        {"g1 vel     [294:584]",      294,  584,  false},
        {"g1 anchor  [584:644]",      584,  644,  false},
        {"tel anchor [644:650]",      644,  650,  false},
        {"tel lower  [650:890]",      650,  890,  false},
        {"vr3        [890:911]",      890,  911,  false},
        {"smpl joints[911:1631]",     911,  1631, true},
        {"smpl anchor[1631:1691]",    1631, 1691, true},
        {"smpl wrists[1691:1751]",    1691, 1751, true},
    };
    std::memcpy(probe.input(), obs.data(), obs.size() * sizeof(float));
    if (!probe.Infer()) return 1;
    std::vector<float> base_tok(probe.output(), probe.output() + TokenEncoder::kTokenDim);
    std::printf("baseline re-infer vs step token: max diff %.3e\n",
                max_abs_diff(base_tok.data(), token.data(), token.size()));

    bool ok = true;
    for (const auto& r : regions) {
        std::memcpy(probe.input(), obs.data(), obs.size() * sizeof(float));
        for (size_t i = r.lo; i < r.hi; ++i) probe.input()[i] += 0.1f;
        if (!probe.Infer()) return 1;
        double d = max_abs_diff(probe.output(), base_tok.data(), base_tok.size());
        bool live = d > 1e-6;
        bool pass = (live == r.expect_live);
        ok = ok && pass;
        std::printf("  %-24s token max delta %.3e  -> %s  %s\n", r.name, d,
                    live ? "LIVE" : "dead", pass ? "" : "<<< UNEXPECTED");
    }
    // mode byte sanity: same data tagged teleop(1) must give a different token
    std::memcpy(probe.input(), obs.data(), obs.size() * sizeof(float));
    probe.input()[0] = 1.0f;
    if (!probe.Infer()) return 1;
    double dm = max_abs_diff(probe.output(), base_tok.data(), base_tok.size());
    std::printf("  mode byte 2 -> 1 (same data): token max delta %.3e -> %s\n", dm,
                dm > 1e-6 ? "differs (ok)" : "SAME <<< UNEXPECTED");
    ok = ok && dm > 1e-6;

    std::printf("perturbation scan: %s\n", ok ? "PASS" : "FAIL");

    return ok ? 0 : 1;
}
