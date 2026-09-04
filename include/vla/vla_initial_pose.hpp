#pragma once

#include <array>

namespace kist {

// 64-dim SONIC motion token for a stable standing pose — SONIC v1.1 space.
// Used as the safe hold target when the VLA token stream is lost.
//
// Source: derived 2026-08-31 by encoding the measured standby stance of
// collector session 20260831_080750 with the v1.1 encoder (10-frame hold,
// zero velocities, identity heading-relative anchor). Cross-checked against
// gearsonic's own live-encoded standby tokens recorded in that session.
// Identical to kist-vla-inference's DEFAULT_INITIAL_MOTION_TOKEN
// (src/common/config.py; tests/derive_standing_token.py re-derives both).
//
// NOT YET hardware-verified as the recovery target: confirm the replay
// lead-in stands well with this stance before merging.
//
// CHECKPOINT-SPECIFIC: a different SONIC checkpoint encodes a different
// latent space — this token MUST be re-derived if the SONIC checkpoint
// (models/model_*.onnx) changes. Previous values are in git history.
inline constexpr std::array<float, 64> kVlaSafeStandingToken = {
     0.0000f, -0.3750f, -0.1250f, -0.1250f,  0.1250f,  0.1250f, -0.0625f,
    -0.0625f, -0.2500f, -0.0625f, -0.1875f, -0.0625f,  0.3750f,  0.1250f,
     0.1250f,  0.0625f, -0.1250f, -0.1250f, -0.1250f,  0.1250f, -0.0625f,
    -0.0625f,  0.0000f,  0.2500f, -0.4375f,  0.2500f, -0.1250f,  0.0625f,
     0.1875f, -0.2500f,  0.0000f,  0.1250f,  0.0000f,  0.0000f,  0.2500f,
     0.0000f, -0.1250f, -0.0625f,  0.1250f, -0.0625f, -0.2500f,  0.1875f,
    -0.0625f,  0.1250f,  0.0000f,  0.4375f,  0.3750f,  0.0000f,  0.2500f,
    -0.1250f, -0.0625f, -0.0625f, -0.3125f, -0.1250f,  0.1250f, -0.1875f,
     0.4375f, -0.0625f,  0.0625f, -0.0625f, -0.2500f,  0.0625f, -0.1875f,
     0.1250f,
};

}  // namespace kist
