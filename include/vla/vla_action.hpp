#pragma once

#include <array>
#include <cstdint>

namespace kist {

// DDS topics shared with kist-vla-inference — must match the constants in
// its kist_vla/io/dds.py. Named in the rt/kist/* convention used by
// kist-ext-sensor-io. (Wire types: idl/kist_latent_action.idl.)
inline constexpr const char* kVlaLatentActionTopic = "rt/kist/latent_action";
inline constexpr const char* kVlaWbcCommandTopic   = "rt/kist/wbc_command";

// Freshness thresholds for the external token stream (see
// WholeBodyController::tick_vla_control):
//   fresh  — a token this recent switches the controller into VLA mode
//   hold   — beyond this age in VLA mode, the stream counts as LOST; the
//            controller then blends to the safe standing token and holds
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

// Snapshot of the newest kist_msgs::WbcCommand.
struct VlaCommand {
    bool     start{false};
    bool     stop{false};
    bool     planner{false};
    uint64_t seq{0};
};

} // namespace kist
