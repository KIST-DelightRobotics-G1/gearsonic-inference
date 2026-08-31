#include "control/token_encoder.hpp"
#include "common/math_utils.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace kist {

// Encoder buffer offsets — SONIC v1.1 layout (sonic_v1_1/observation_config.yaml
// order; see header). Cross-checked 2026-08-28 by perturbation-probing the
// v1.1 ONNX graph's live input regions. v1.1 dropped the release layout's
// unused [584:595] gap (hence 1751 vs 1762) and moved the single-frame
// anchor behind the g1 anchor block, so every offset from 584 on shifted.
static constexpr size_t kOffEncoderMode  = 0;
static constexpr size_t kOffMotionQ      = 4;    // g1 mode
static constexpr size_t kOffMotionDq     = 294;
static constexpr size_t kOffAnchorOri    = 584;  // g1 mode      (release: 601)
static constexpr size_t kOffAnchorSingle = 644;  // teleop mode  (release: 595)
static constexpr size_t kOffLowerQ       = 650;  // teleop mode  (release: 661)
static constexpr size_t kOffLowerDq      = 770;  //              (release: 781)
static constexpr size_t kOffVR3Pos       = 890;  //              (release: 901)
static constexpr size_t kOffVR3Orn       = 899;  //              (release: 910)

static constexpr int kNumJoints  = MotionSequence50Hz::kNumJoints;
static constexpr int kFutFrames  = 10;   // *_10frame_step5
static constexpr int kFutStep    = 5;

// Lower-body joints (mujoco order) as indexes into the IsaacLab-ordered
// motion joints (gear_sonic lower_body_joint_mujoco_order_in_isaaclab_index)
static constexpr int kNumLowerJoints = 12;
static constexpr int kLowerBodyIdx[kNumLowerJoints] = {0, 3, 6, 9, 13, 17,
                                                       1, 4, 7, 10, 14, 18};

// Future-frame sampling: hold current frame when not playing, otherwise
// advance by step and clamp to the last frame (hold final pose).
static int target_frame(int current_frame, int frame_idx, int step, int timesteps,
                        bool playing) {
    int f = current_frame;
    if (playing)
        f = std::min(f + frame_idx * step, timesteps - 1);
    return std::clamp(f, 0, timesteps - 1);
}

bool TokenEncoder::init(const std::string& onnx_path) {
    return model_.Initialize(onnx_path, "encoded_tokens", kInputDim, kTokenDim);
}

bool TokenEncoder::step(const MotionSequence50Hz& motion, int cursor, bool playing,
                        const StateLogger& logger, const VR3Point* vr3point,
                        Token& token_out) {
    int mode = vr3point ? 1 : 0;
    if (mode != last_mode_) {
        std::cout << "[TokenEncoder] encoder mode -> "
                  << (mode == 1 ? "teleop(1)" : "g1(0)") << "\n";
        last_mode_ = mode;
    }

    update_heading_state(motion, cursor, logger);
    fill_obs(model_.input(), motion, cursor, playing, logger, vr3point);

    if (!model_.Infer())
        return false;

    std::memcpy(token_out.data(), model_.output(), kTokenDim * sizeof(float));
    return true;
}

void TokenEncoder::update_heading_state(const MotionSequence50Hz& motion,
                                        int cursor,
                                        const StateLogger& logger) {
    if (motion.timesteps == 0)
        return;

    if (reinitialize_heading_) {
        init_base_quat_ = logger.GetLatest(1)[0].base_quat;
        delta_heading_  = 0.0;
        init_ref_root_rot_ = motion.frames[std::clamp(cursor, 0, motion.timesteps - 1)].quaternion;
        reinitialize_heading_ = false;
        std::cout << "[TokenEncoder] heading state reset\n";
    }

    if (cursor == 0)
        init_ref_root_rot_ = motion.frames[0].quaternion;
}

std::array<double, 4> TokenEncoder::compute_apply_delta_heading() const {
    auto init_heading     = calc_heading_quat(init_base_quat_);
    auto data_heading_inv = calc_heading_quat_inv(init_ref_root_rot_);
    auto apply            = quat_mul(init_heading, data_heading_inv);
    if (delta_heading_ != 0.0)
        apply = quat_mul(euler_z_to_quat(delta_heading_), apply);
    return apply;
}

// Anchor orientation for N future frames (orientation_mode 0: full robot
// base quat as left side); 6 values per frame — first two rotation-matrix
// columns, flattened row-wise.
void TokenEncoder::fill_anchor_orientation(float* out, int num_frames, int step,
                                           const MotionSequence50Hz& motion,
                                           int cursor, bool playing,
                                           const std::array<double, 4>& base_quat) const {
    auto apply_delta_heading = compute_apply_delta_heading();

    for (int f = 0; f < num_frames; ++f) {
        int tf = target_frame(cursor, f, step, motion.timesteps, playing);
        auto new_ref_root_rot = quat_mul(apply_delta_heading, motion.frames[tf].quaternion);
        auto base_to_ref      = quat_mul(quat_conjugate(base_quat), new_ref_root_rot);
        auto m                = quat_to_rotation_matrix(base_to_ref);

        float* o = out + f * 6;
        o[0] = static_cast<float>(m[0][0]); o[1] = static_cast<float>(m[0][1]);
        o[2] = static_cast<float>(m[1][0]); o[3] = static_cast<float>(m[1][1]);
        o[4] = static_cast<float>(m[2][0]); o[5] = static_cast<float>(m[2][1]);
    }
}

void TokenEncoder::fill_obs(float* dst, const MotionSequence50Hz& motion,
                            int cursor, bool playing, const StateLogger& logger,
                            const VR3Point* vr3point) const {
    std::memset(dst, 0, kInputDim * sizeof(float));
    if (motion.timesteps == 0)
        return;

    auto base_quat = logger.GetLatest(1)[0].base_quat;

    if (!vr3point) {
        // ── g1 (0): planner motion drives the whole body ──────────
        dst[kOffEncoderMode] = 0.0f;

        for (int f = 0; f < kFutFrames; ++f) {
            int tf = target_frame(cursor, f, kFutStep, motion.timesteps, playing);
            const auto& frame = motion.frames[tf];
            for (int j = 0; j < kNumJoints; ++j) {
                dst[kOffMotionQ  + f * kNumJoints + j] = static_cast<float>(frame.joints[j]);
                // gear_sonic zero-fills velocities while not playing
                dst[kOffMotionDq + f * kNumJoints + j] =
                    playing ? static_cast<float>(frame.joint_velocities[j]) : 0.0f;
            }
        }
        fill_anchor_orientation(dst + kOffAnchorOri, kFutFrames, kFutStep,
                                motion, cursor, playing, base_quat);
        return;
    }

    // ── teleop (1): planner lower body + VR 3-point upper body ────
    dst[kOffEncoderMode] = 1.0f;

    for (int f = 0; f < kFutFrames; ++f) {
        int tf = target_frame(cursor, f, kFutStep, motion.timesteps, playing);
        const auto& frame = motion.frames[tf];
        for (int j = 0; j < kNumLowerJoints; ++j) {
            int idx = kLowerBodyIdx[j];
            dst[kOffLowerQ  + f * kNumLowerJoints + j] = static_cast<float>(frame.joints[idx]);
            dst[kOffLowerDq + f * kNumLowerJoints + j] =
                playing ? static_cast<float>(frame.joint_velocities[idx]) : 0.0f;
        }
    }
    fill_anchor_orientation(dst + kOffAnchorSingle, 1, 1,
                            motion, cursor, playing, base_quat);

    for (int i = 0; i < 9; ++i)
        dst[kOffVR3Pos + i] = static_cast<float>(vr3point->position[i]);
    for (int i = 0; i < 12; ++i)
        dst[kOffVR3Orn + i] = static_cast<float>(vr3point->orientation[i]);
}

} // namespace kist
