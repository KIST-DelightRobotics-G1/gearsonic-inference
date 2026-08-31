#pragma once

#include "control/obs_dict_model.hpp"
#include "control/state_logger.hpp"
#include "planner/motion_sequence_50hz.hpp"
#include "teleop/vr_3point.hpp"

#include <array>
#include <atomic>
#include <string>

namespace kist {

// Encoder stage: reference motion (+ optional VR 3-point) -> token.
//
// Owns the encoder TRT model, the observation assembly for it, and the
// heading-alignment state. Pure computation: inputs are data only — the
// orchestrator (WholeBodyController) decides when to run it, and
// operational signals (e-stop, safety) never enter this class.
//
// Encoder obs_dict [1751] — SONIC v1.1 (sonic_v1_1/observation_config.yaml);
// the mode is chosen per tick by the presence of VR 3-point data:
//
// g1 (0) — planner drives the whole body:
//   [0:4]     encoder_mode_4            (mode id, then zeros)
//   [4:294]   motion_joint_positions_10frame_step5   (IsaacLab order)
//   [294:584] motion_joint_velocities_10frame_step5
//   [584:644] motion_anchor_orientation_heading_10frame_step5
//
// teleop (1) — planner drives the lower body, VR 3-point the upper:
//   [0:4]     encoder_mode_4
//   [644:650] motion_anchor_orientation_heading  (single frame)
//   [650:770] motion_joint_positions_lowerbody_10frame_step5
//   [770:890] motion_joint_velocities_lowerbody_10frame_step5
//   [890:899] vr_3point_local_target
//   [899:911] vr_3point_local_orn_target
//
// Slots of the other modes stay zero. (The release checkpoint used dim 1762
// with the anchor at [601:661]; v1.1 removed the unused [584:595] gap.)
class TokenEncoder {
public:
    static constexpr size_t kInputDim = 1751;
    static constexpr size_t kTokenDim = 64;
    using Token = std::array<float, kTokenDim>;

    bool init(const std::string& onnx_path);

    // One control tick: heading-state update, observation fill, inference.
    // vr3point selects the encoder mode: nullptr -> g1(0), data -> teleop(1).
    // Fails only on inference errors.
    bool step(const MotionSequence50Hz& motion, int cursor, bool playing,
              const StateLogger& logger, const VR3Point* vr3point,
              Token& token_out);

    // Encoder mode used by the last step (g1=0, teleop=1; -1 before first)
    int mode() const { return last_mode_; }

    // Re-anchor the heading alignment (called when a fresh playback
    // timeline starts, e.g. the first planner motion is adopted).
    void request_heading_reinit() { reinitialize_heading_ = true; }

private:
    void update_heading_state(const MotionSequence50Hz& motion, int cursor,
                              const StateLogger& logger);
    void fill_obs(float* dst, const MotionSequence50Hz& motion, int cursor,
                  bool playing, const StateLogger& logger,
                  const VR3Point* vr3point) const;
    void fill_anchor_orientation(float* out, int num_frames, int step,
                                 const MotionSequence50Hz& motion, int cursor,
                                 bool playing,
                                 const std::array<double, 4>& base_quat) const;
    std::array<double, 4> compute_apply_delta_heading() const;

    ObsDictModel model_;

    std::atomic<int> last_mode_{-1};

    // Heading alignment state (gear_sonic HeadingState + init ref root rot)
    std::array<double, 4> init_base_quat_{1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> init_ref_root_rot_{1.0, 0.0, 0.0, 0.0};
    double                delta_heading_{0.0};
    bool                  reinitialize_heading_{true};
};

} // namespace kist
