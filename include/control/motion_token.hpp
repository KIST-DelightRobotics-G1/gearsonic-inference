#pragma once

#include <array>
#include <cstdint>

namespace kist {

// One CONTROL tick's decoder-input token, tapped by WholeBodyController
// into motion_token_buf for the recording stream (rt/kist/motion_token —
// wire type kist_msgs::MotionTokenState, published by MotionTokenPublisher).
//
// seq increments only on ticks that actually decoded a token: gaps mark
// non-CONTROL periods (INIT, damping, e-stop, missing planner motion).
// stamp_ns is the computation time (epoch), not the publish time, so the
// collector aligns records to the tick that produced them.
struct MotionTokenSample {
    std::array<float, 64> token{};
    uint64_t seq{0};
    int64_t  stamp_ns{0};
    uint8_t  arbiter_mode{0};    // ControlArbiter::Mode value
    uint8_t  encoder_mode{255};  // 0 g1 / 1 teleop / 255 = encoder skipped
};

inline constexpr const char* kMotionTokenTopic = "rt/kist/motion_token";

} // namespace kist
