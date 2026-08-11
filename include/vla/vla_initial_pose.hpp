#pragma once

#include <array>

namespace kist {

// 64-dim SONIC motion token for a stable standing pose (arms in the ready
// position). Used as the safe hold target when the VLA token stream is lost.
//
// Source: NVIDIA GR00T-WholeBodyControl gear_sonic/utils/inference/
// initial_poses.py (LATENT_INITIAL_MOTION_TOKEN) — the same value
// kist-vla-inference uses for its initial pose (kist_vla/config.py; keep in
// sync). Empirically verified on the real G1 against the public GEAR-SONIC
// checkpoint (2026-08-10): the robot stands with arms spread ~90°.
//
// CHECKPOINT-SPECIFIC: a different SONIC checkpoint encodes a different
// latent space — this token MUST be re-derived if the SONIC checkpoint
// (models/model_*.onnx) changes.
inline constexpr std::array<float, 64> kVlaSafeStandingToken = {
    -0.0625f,  0.0000f, -0.0625f, -0.1250f, -0.1875f, -0.0625f,  0.1875f,
     0.2500f,  0.1875f, -0.1250f,  0.0625f, -0.0625f, -0.2500f, -0.2500f,
    -0.3125f, -0.0625f,  0.0000f, -0.0625f, -0.1250f, -0.1875f,  0.0000f,
    -0.2500f,  0.0000f, -0.2500f, -0.0625f,  0.0625f,  0.1250f, -0.1250f,
     0.2500f,  0.1875f,  0.2500f, -0.1250f,  0.1250f,  0.1875f, -0.0625f,
     0.0000f, -0.1875f, -0.1875f,  0.2500f,  0.0000f,  0.0000f, -0.1250f,
     0.0625f,  0.0000f, -0.0625f, -0.0625f,  0.1875f, -0.0625f,  0.0000f,
     0.0625f,  0.1250f,  0.0625f,  0.1250f,  0.0625f,  0.1250f,  0.0000f,
     0.1250f,  0.1875f,  0.0000f,  0.0000f,  0.0625f,  0.0625f,  0.1875f,
     0.0625f,
};

} // namespace kist
