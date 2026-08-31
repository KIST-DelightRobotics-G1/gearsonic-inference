#pragma once

#include <array>

namespace kist {

// 64-dim SONIC motion token for a stable standing pose — SONIC v1.1 space.
// Used as the safe hold target when the VLA token stream is lost.
//
// Source: derived 2026-08-28 by encoding a real recorded standing pose with
// the v1.1 encoder (sonic_v1_1/model_encoder.onnx, obs_dict 1751, g1 mode):
// the calmest 60-tick stationary window of a collected episode
// (episode_000003, sway 0.031 rad), held constant for the 10 future frames,
// zero velocities, identity heading-relative anchor. Values land exactly on
// the FSQ 1/16 grid and are window-stable to <=1 grid step.
// (kist-vla-inference tests/derive_standing_token.py re-derives it.)
//
// NOT YET hardware-verified: before trusting the recovery path, stream this
// token (publish_test_tokens.py) and confirm the robot stands.
//
// CHECKPOINT-SPECIFIC: a different SONIC checkpoint encodes a different
// latent space — this token MUST be re-derived if the SONIC checkpoint
// (models/model_*.onnx) changes. The previous (release-checkpoint) value is
// in git history and NVIDIA's gear_sonic/utils/inference/initial_poses.py.
inline constexpr std::array<float, 64> kVlaSafeStandingToken = {
     0.2500f, -0.3125f, -0.0625f,  0.0000f, -0.0625f, -0.0625f,  0.1250f,
     0.0625f, -0.2500f,  0.0625f,  0.0000f, -0.1250f,  0.4375f,  0.0000f,
     0.1250f,  0.0625f,  0.0625f,  0.0000f,  0.0000f, -0.0625f,  0.0000f,
     0.0000f, -0.1250f,  0.1250f, -0.3750f,  0.3125f, -0.1250f,  0.0000f,
     0.2500f, -0.4375f,  0.1250f, -0.0625f, -0.0625f,  0.1875f,  0.3750f,
     0.0625f, -0.1250f,  0.1875f,  0.1250f, -0.1250f, -0.1250f,  0.1250f,
     0.1250f, -0.3125f,  0.2500f,  0.4375f,  0.5000f, -0.1875f, -0.1250f,
    -0.1875f,  0.0000f,  0.0000f, -0.4375f, -0.1875f,  0.1250f, -0.0625f,
     0.4375f,  0.1250f,  0.1250f,  0.0000f, -0.1250f,  0.0000f,  0.0625f,
     0.0000f,
};

}  // namespace kist
