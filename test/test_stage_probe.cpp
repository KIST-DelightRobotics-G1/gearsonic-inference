// Deterministic single-thread probe of the encoder/decoder stages.
// Fixed synthetic inputs -> prints token / raw action / q_target at high
// precision. Used to prove refactors don't change the computation:
// run before and after, outputs must match to the last digit.

#include "common/config.hpp"
#include "common/joint_order.hpp"
#include "common/robot_params.hpp"
#include "control/policy_decoder.hpp"
#include "control/token_encoder.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using namespace kist;

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    auto cfg = Config::instance().root()["control"];

    TokenEncoder encoder;
    PolicyDecoder decoder;
    if (!encoder.init(cfg["encoder_path"].as<std::string>()))
        return 1;
    if (!decoder.init(cfg["decoder_path"].as<std::string>()))
        return 1;

    // Synthetic robot-state history: 10 deterministic entries
    StateLogger logger;
    for (int k = 0; k < 10; ++k) {
        StateLogger::Entry e;
        double a = 0.05 * k;
        e.base_quat = {std::cos(a / 2), 0.0, 0.0, std::sin(a / 2)};
        e.base_ang_vel = {0.01 * k, -0.02 * k, 0.005 * k};
        for (int j = 0; j < 29; ++j) {
            e.body_q[j]      = 0.02 * std::sin(0.3 * j + 0.1 * k);
            e.body_dq[j]     = 0.10 * std::cos(0.2 * j - 0.1 * k);
            e.last_action[j] = 0.05 * std::sin(0.1 * j * k);
        }
        logger.Log(e);
    }

    // Synthetic motion: 60 frames, smooth deterministic curves
    MotionSequence50Hz motion;
    motion.resize(60);
    for (int f = 0; f < 60; ++f) {
        auto& fr = motion.frames[f];
        double t = f / 50.0;
        fr.position = {0.4 * t, 0.05 * std::sin(t), 0.78 + 0.01 * std::cos(t)};
        double yaw = 0.2 * t;
        fr.quaternion = {std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)};
        for (int j = 0; j < 29; ++j) {
            fr.joints[j]           = 0.3 * std::sin(2.0 * t + 0.4 * j);
            fr.joint_velocities[j] = 0.6 * std::cos(2.0 * t + 0.4 * j);
        }
    }

    constexpr int kCursor = 5;

    // ── encoder stage, g1 mode (no VR 3-point) ──
    TokenEncoder::Token token;
    if (!encoder.step(motion, kCursor, /*playing=*/true, logger, nullptr, nullptr, token))
        return 1;

    std::printf("token   :");
    for (int i = 0; i < 8; ++i) std::printf(" %.6f", token[i]);
    std::printf("\n");

    // ── decoder stage ──
    MotorCommand cmd;
    if (!decoder.step(token, logger, cmd))
        return 1;

    const auto& action = decoder.last_action();
    std::printf("action  :");
    for (int i = 0; i < 8; ++i) std::printf(" %.6f", action[i]);
    std::printf("\n");

    std::printf("q_target:");
    for (int i = 0; i < 8; ++i) {
        double q = g1_default_angles[i] + action[isaaclab_to_mujoco[i]] * g1_action_scale[i];
        std::printf(" %.6f", q);
    }
    std::printf("\n");

    // ── encoder stage, teleop mode (synthetic VR 3-point) ──
    VR3Point vr3;
    for (int i = 0; i < 9; ++i)
        vr3.position[i] = 0.30 + 0.05 * std::sin(0.7 * i);
    for (int k = 0; k < 3; ++k) {
        double ang = 0.2 + 0.15 * k;
        vr3.orientation[k * 4 + 0] = std::cos(ang / 2);
        vr3.orientation[k * 4 + 1] = 0.0;
        vr3.orientation[k * 4 + 2] = 0.0;
        vr3.orientation[k * 4 + 3] = std::sin(ang / 2);
    }

    TokenEncoder::Token token1;
    if (!encoder.step(motion, kCursor, /*playing=*/true, logger, &vr3, nullptr, token1))
        return 1;

    std::printf("token1  :");
    for (int i = 0; i < 8; ++i) std::printf(" %.6f", token1[i]);
    std::printf("\n");
    return 0;
}
