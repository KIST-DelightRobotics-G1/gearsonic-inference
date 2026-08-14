#pragma once

#include <array>
#include <cstdint>

namespace kist {

// DDS topic shared with kist-vla-inference — must match the constants in
// its kist_vla/io/dds.py. Named in the rt/kist/* convention used by
// kist-ext-sensor-io. (Wire types: idl/kist_msgs.idl. The
// rt/kist/wbc_command topic defined there is RESERVED for the future
// Orchestrator's lifecycle commands — no subscriber here yet; session
// lifecycle currently rides the data plane: auto-start, stop-publishing ->
// standing hold, VR e-stop -> damping.)
inline constexpr const char* kVlaLatentActionTopic = "rt/kist/latent_action";

// Freshness thresholds for the external token stream (consumed by
// WholeBodyController::update_arbiter, see control_arbiter.hpp):
//   fresh  — a token this recent claims the robot for VLA (from the origin)
//   hold   — beyond this age in VLA mode, the stream counts as LOST; the
//            controller blends to the safe standing token, reseeds the
//            planner, and returns to the origin
//            (kVlaSafeStandingToken; ticks below set the blend length)
inline constexpr double kVlaTokenFreshMs    = 200.0;
inline constexpr double kVlaTokenHoldMs     = 500.0;
inline constexpr int    kVlaLossBlendTicks  = 50;  // 1s at 50Hz

// Plain-data snapshot of the newest kist_msgs::LatentActionStep.
// token drives the whole-body decoder; the hand joints are direct Dex3
// targets in motor order (thumb x3, index x2, middle x2) — the same layout
// HandCommand::q uses.
struct VlaLatentAction {
    std::array<float, 64> token{};
    std::array<float, 7>  left_hand{};
    std::array<float, 7>  right_hand{};
    int64_t  frame_index{0};
    int64_t  stamp_ns{0};
    uint64_t seq{0};
};

} // namespace kist
