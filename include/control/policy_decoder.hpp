#pragma once

#include "control/motor_command.hpp"
#include "control/obs_dict_model.hpp"
#include "control/state_logger.hpp"
#include "control/token_encoder.hpp"

#include <array>
#include <string>

namespace kist {

// Decoder (policy) stage: token + robot-state history -> motor command.
//
// Owns the decoder TRT model, its observation assembly, the raw-action
// memory (fed back as the his_last_actions observation), and the action
// post-processing. Pure computation, like TokenEncoder.
//
// Policy obs_dict [994] (observation_config.yaml offsets):
//   [0:64]    token_state (from TokenEncoder)
//   [64:94]   his_base_angular_velocity_10frame_step1
//   [94:384]  his_body_joint_positions_10frame_step1
//   [384:674] his_body_joint_velocities_10frame_step1
//   [674:964] his_last_actions_10frame_step1
//   [964:994] his_gravity_dir_10frame_step1
// History frames are oldest -> newest.
class PolicyDecoder {
public:
    static constexpr size_t kInputDim = 994;

    bool init(const std::string& onnx_path);
    void release() { model_.release(); }

    // One control tick: observation fill, inference, post-processing
    // (q = default + scale * remap(action)). Retains the raw action.
    bool step(const TokenEncoder::Token& token, const StateLogger& logger,
              MotorCommand& cmd_out);

    // Raw policy output (IsaacLab order) — logged into the state history
    // by the orchestrator, which the policy reads back as an observation.
    const std::array<double, 29>& last_action() const { return last_action_; }

private:
    void fill_obs(float* dst, const StateLogger& logger) const;

    ObsDictModel model_;
    std::array<double, 29> last_action_{};
};

} // namespace kist
