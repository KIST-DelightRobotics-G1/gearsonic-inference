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
//   0..2  -> thumb   (3 DOF, bidirectional joints — matches SDK limit
//                    signs: ±1.05, ±0.72, ±1.75)
//   3..4  -> index   (2 DOF, unidirectional flexion — 0..±1.57 / ±1.75)
//   5..6  -> middle  (2 DOF, unidirectional flexion)
// The split lets the two controller analog axes drive anatomically
// aligned finger groups: grip -> thumb, trigger -> index+middle.
inline constexpr int kThumbBegin  = 0;
inline constexpr int kFingerBegin = 3;
inline constexpr int kMotorEnd    = 7;

// Explicit open/closed endpoints (empirically confirmed on the real hand):
//   thumb (0..2) is bidirectional — open/close sit on opposite signs;
//   fingers (3..6) are unidirectional flexion — 0 is extended (open),
//   the signed extreme is curled (closed). Both hands share the same
//   convention (0 = open) for the fingers; only the sign flips.
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
inline constexpr std::array<float, 7> kDex3LeftOpen  = { 0.00f, -0.724f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f};
inline constexpr std::array<float, 7> kDex3LeftClose = { 0.00f,  1.05f,   1.75f, -1.57f, -1.75f, -1.57f, -1.75f};
inline constexpr std::array<float, 7> kDex3RightOpen  = { 0.00f,  0.742f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f};
inline constexpr std::array<float, 7> kDex3RightClose = { 0.00f, -1.05f,  -1.75f,  1.57f,  1.75f,  1.57f,  1.75f};

} // namespace kist
