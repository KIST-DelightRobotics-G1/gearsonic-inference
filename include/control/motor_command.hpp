#pragma once

#include <array>

namespace kist {

// Per-motor PD command, MuJoCo/DDS motor order.
// Produced by the 50Hz WholeBodyController into a DataBuffer<MotorCommand>;
// consumed by the 500Hz UnitreeCommandWriter (rt/lowcmd).
struct MotorCommand {
    std::array<float, 29> q_target{};   // target joint position (rad)
    std::array<float, 29> dq_target{};  // target joint velocity (0, matches gear_sonic)
    std::array<float, 29> kp{};         // position gain
    std::array<float, 29> kd{};         // damping gain
    std::array<float, 29> tau_ff{};     // feed-forward torque (0)
};

} // namespace kist
