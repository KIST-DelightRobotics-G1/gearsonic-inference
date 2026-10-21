#pragma once

#include <array>

namespace kist {

// Dex3-1 has 7 motors per hand. Command layout follows the SDK example
// (unitree_sdk2 g1_dex3_example): q + kp + kd per motor. Left/right are
// separate topics (rt/dex3/{left,right}/cmd), so each hand carries its own
// command; a single writer publishes both.
struct HandCommand {
    std::array<float, 7> q{};
    std::array<float, 7> kp{};
    std::array<float, 7> kd{};
    bool enabled{true};  // false -> RIS_Mode timeout bit set (safe stop)
};

// Dex3-1 is a 3-finger gripper (thumb + index + middle) with 7 motors:
//   0     -> thumb rotation (±1.05, symmetric on both hands)
//   1     -> thumb bend     (bidirectional, asymmetric limits per hand)
//   2     -> thumb flexion  (unidirectional, 0..1.75 L / -1.75..0 R)
//   3..4  -> index   (2 DOF, unidirectional flexion — 0..±1.57 / ±1.75)
//   5..6  -> middle  (2 DOF, unidirectional flexion)
// The two controller analog axes drive the finger groups the way the
// operator's own fingers sit on the controller: the index finger rests on
// the TRIGGER, the middle finger squeezes the GRIP. So trigger -> index,
// grip -> thumb + middle. Grip alone therefore closes thumb and middle
// around an extended index finger — the pointing gesture — and both axes
// together make a full fist.
inline constexpr int kThumbBegin  = 0;
inline constexpr int kIndexBegin  = 3;
inline constexpr int kMiddleBegin = 5;
inline constexpr int kMotorEnd    = 7;

// Controller axis for motor i: true = trigger (index), false = grip.
inline constexpr bool motor_on_trigger(int i) { return i >= kIndexBegin && i < kMiddleBegin; }

// Explicit open/closed endpoints (empirically confirmed on the real hand):
//   the thumb bend (1) is bidirectional — open/close sit on opposite
//   signs; the thumb flexion (2) and the fingers (3..6) are unidirectional
//   — 0 is extended (open), the signed extreme is curled (closed). Both
//   hands share the same convention (0 = open); only the sign flips.
//
// URDF joint limits from unitree_sdk2 example/g1/dex3/g1_dex3_example.cpp
// are kept for reference — the endpoints below lie within them.
//   Left  min = {-1.05, -0.724,  0.00, -1.57, -1.75, -1.57, -1.75}
//   Left  max = { 1.05,  1.05,   1.75,  0.00,  0.00,  0.00,  0.00}
//   Right min = {-1.05, -1.05,  -1.75,  0.00,  0.00,  0.00,  0.00}
//   Right max = { 1.05,  0.742,  0.00,  1.57,  1.75,  1.57,  1.75}
//
// Motor 0 (thumb rotation) is pinned at the center of its symmetric
// ±1.05 range on both hands: sweeping it during the grasp visibly twists
// the thumb sideways, so the thumb only flexes (motors 1..2) and never
// rotates.
//
// The thumb's OPEN bend (motor 1) is deliberately short of the URDF limit
// (-0.724 L / 0.742 R): at the limit the thumb splays uselessly far from
// the palm. kThumbOpenBend is the tuning knob — smaller magnitude = a
// narrower resting thumb; the closed endpoint is untouched.
inline constexpr float kThumbOpenBend = 0.35f;
inline constexpr std::array<float, 7> kDex3LeftOpen  = { 0.00f, -kThumbOpenBend, 0.00f,  0.00f,  0.00f,  0.00f,  0.00f};
// Finger closed endpoints follow gear_sonic's Dex3 close pose (proximal
// 1.0, distal 1.5 rad — the tip curls 1.5x more than the base so the
// finger wraps an object instead of clamping it), not the URDF limits
// (1.57 / 1.75). Thumb endpoints are our own, tuned on the real hand.
inline constexpr std::array<float, 7> kDex3LeftClose = { 0.00f,  1.05f,          1.75f, -1.00f, -1.50f, -1.00f, -1.50f};
inline constexpr std::array<float, 7> kDex3RightOpen  = { 0.00f,  kThumbOpenBend, 0.00f,  0.00f,  0.00f,  0.00f,  0.00f};
inline constexpr std::array<float, 7> kDex3RightClose = { 0.00f, -1.05f,         -1.75f,  1.00f,  1.50f,  1.00f,  1.50f};

// Full-fist endpoints: where a finger goes when the operator keeps the
// axis fully pulled (HoldSqueeze below) — the URDF limits for the fingers,
// the same values as kDex3*Close for the thumb. Nothing between kDex3*Close
// and these is reached by the plain 0..1 mapping, so the contact timing of
// the close pose is untouched; the extra travel only adds once the pilot
// holds the axis at the end.
inline constexpr std::array<float, 7> kDex3LeftFist  = { 0.00f,  1.05f,  1.75f, -1.57f, -1.75f, -1.57f, -1.75f};
inline constexpr std::array<float, 7> kDex3RightFist = { 0.00f, -1.05f, -1.75f,  1.57f,  1.75f,  1.57f,  1.75f};

// Per-axis "held at the end" detector. Feed the raw axis each tick; returns
// the squeeze fraction s in [0, 1]: 0 while the axis is below kThreshold,
// then after kDwellS at the end it ramps to 1 over kRampS. Releasing the
// axis drops s to 0 immediately (no ramp back — letting go must be fast).
struct HoldSqueeze {
    static constexpr double kThreshold = 0.95;  // analog axes rarely report exactly 1.0
    static constexpr double kDwellS    = 0.20;  // "grabbed, now squeeze" pause
    static constexpr double kRampS     = 0.30;  // close pose -> fist

    double held_s{0.0};

    double update(double axis, double dt_s) {
        if (axis < kThreshold) { held_s = 0.0; return 0.0; }
        held_s += dt_s;
        double s = (held_s - kDwellS) / kRampS;
        return s < 0.0 ? 0.0 : (s > 1.0 ? 1.0 : s);
    }
    void reset() { held_s = 0.0; }
};

// Trigger / grip -> Dex3 command. t in [0, 1] blends open -> close per
// motor group (trigger: index; grip: thumb + middle); the squeeze fraction
// s (HoldSqueeze) then blends close -> fist on top. s = 0 is the plain
// mapping; s = 1 is a full fist.
inline HandCommand hand_command_from_axes(double grip, double trigger,
                                          double squeeze_grip, double squeeze_trigger,
                                          bool is_left, float kp, float kd) {
    auto clamp01 = [](double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); };
    grip = clamp01(grip); trigger = clamp01(trigger);
    squeeze_grip = clamp01(squeeze_grip); squeeze_trigger = clamp01(squeeze_trigger);
    const auto& open  = is_left ? kDex3LeftOpen  : kDex3RightOpen;
    const auto& close = is_left ? kDex3LeftClose : kDex3RightClose;
    const auto& fist  = is_left ? kDex3LeftFist  : kDex3RightFist;
    HandCommand cmd;
    for (int i = 0; i < kMotorEnd; ++i) {
        bool   on_trig = motor_on_trigger(i);
        double t = on_trig ? trigger : grip;
        double s = on_trig ? squeeze_trigger : squeeze_grip;
        double q = open[i] + t * (close[i] - open[i]) + s * (fist[i] - close[i]);
        cmd.q[i]  = static_cast<float>(q);
        cmd.kp[i] = kp;
        cmd.kd[i] = kd;
    }
    cmd.enabled = true;
    return cmd;
}

// URDF joint limits (the table above), used to clamp external/derived
// targets before publishing.
inline constexpr std::array<float, 7> kDex3LeftMin  = {-1.05f, -0.724f,  0.00f, -1.57f, -1.75f, -1.57f, -1.75f};
inline constexpr std::array<float, 7> kDex3LeftMax  = { 1.05f,  1.05f,   1.75f,  0.00f,  0.00f,  0.00f,  0.00f};
inline constexpr std::array<float, 7> kDex3RightMin = {-1.05f, -1.05f,  -1.75f,  0.00f,  0.00f,  0.00f,  0.00f};
inline constexpr std::array<float, 7> kDex3RightMax = { 1.05f,  0.742f,  0.00f,  1.57f,  1.75f,  1.57f,  1.75f};

} // namespace kist
