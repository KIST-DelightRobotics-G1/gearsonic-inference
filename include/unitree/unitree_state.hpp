#pragma once

#include "unitree/imu.hpp"
#include <array>
#include <cstdint>

namespace kist {

constexpr int kNumMotors = 29;  // Unitree G1

struct MotorState {
    double   q{0.0};          // position (rad)
    double   dq{0.0};         // velocity (rad/s)
    double   tau{0.0};        // estimated torque (Nm)
    int16_t  temp_casing{0};  // temperature[0]: motor casing (SDK limit 85C)
    int16_t  temp_winding{0}; // temperature[1]: motor winding (SDK limit 120C)
    uint32_t error{0};        // SDK motorstate fault word (0 = healthy)
};

struct UnitreeState {
    std::array<MotorState, kNumMotors> motors{};
    IMU      imu_pelvis{};    // pelvis IMU (from LowState.imu_state)
    uint32_t tick{0};
    uint8_t  mode_machine{0}; // robot variant code (echoed back in LowCmd)
};

} // namespace kist
