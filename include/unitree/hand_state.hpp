#pragma once

#include <array>
#include <cstdint>

namespace kist {

// Dex3-1 per-hand feedback (rt/dex3/{left,right}/state). Motor order is
// the command order in hand_command.hpp: thumb x3, index x2, middle x2.
constexpr int kHandMotors = 7;

struct HandMotorState {
    float   q{0.0f};        // position (rad)
    float   dq{0.0f};       // velocity (rad/s)
    float   tau{0.0f};      // estimated torque (Nm)
    int16_t temperature{0}; // hottest of the two reported sensors (deg C)
};

struct HandState {
    std::array<HandMotorState, kHandMotors> motors{};
    float    power_a{0.0f};  // hand bus current (A) — overload signal
    uint32_t error{0};       // error word 0 from the hand controller
};

} // namespace kist
