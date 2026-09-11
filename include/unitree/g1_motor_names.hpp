#pragma once

#include <array>

namespace kist {

// G1 29-DOF motor names in MuJoCo/DDS motor order (rt/lowstate index).
inline constexpr std::array<const char*, 29> kG1MotorNames = {
    "left_hip_pitch",  "left_hip_roll",  "left_hip_yaw",  "left_knee",  "left_ankle_pitch",  "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch",  "left_shoulder_roll",  "left_shoulder_yaw",  "left_elbow",
    "left_wrist_roll",  "left_wrist_pitch",  "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
};

inline const char* g1_motor_name(int i) {
    return (i >= 0 && i < 29) ? kG1MotorNames[i] : "?";
}

} // namespace kist
