#pragma once

#include "control/obs_dict_model.hpp"
#include "control/state_logger.hpp"
#include "planner/motion_sequence_50hz.hpp"
#include "teleop/smpl_pose_window.hpp"
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
// the mode is chosen per tick by which teleop buffer is present (smpl wins
// over 3-point; TeleopTracker never publishes both):
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
// smpl (2) — the operator's whole body (SmplPoseWindow, 10 frames step 1):
//   [0:4]       encoder_mode_4
//   [911:1631]  smpl_joints_10frame_step1          (24 x 3 per frame, frame-major)
//   [1631:1691] smpl_anchor_orientation_heading_10frame_step1  (6D per frame)
//   [1691:1751] motion_joint_positions_wrists_10frame_step1     (q[23..28] per frame)
//
// Slots of the other modes stay zero. (The release checkpoint used dim 1762
// with the anchor at [601:661]; v1.1 removed the unused [584:595] gap.)
// All anchor observations are the *_heading_* variants: the reference root
// is expressed against the robot's yaw only (upstream orientation_mode 1).
class TokenEncoder {
public:
    static constexpr size_t kInputDim = 1751;
    static constexpr size_t kTokenDim = 64;
    using Token = std::array<float, kTokenDim>;

    bool init(const std::string& onnx_path);

    // One control tick: heading-state update, observation fill, inference.
    // Mode: smpl -> 2, else vr3point -> 1, else g1 (0). Fails only on
    // inference errors.
    bool step(const MotionSequence50Hz& motion, int cursor, bool playing,
              const StateLogger& logger, const VR3Point* vr3point,
              const SmplPoseWindow* smpl, Token& token_out);

    // Encoder mode used by the last step (g1=0, teleop=1, smpl=2; -1 before first)
    int mode() const { return last_mode_; }

    // The observation vector the last step() assembled — read-only, for the
    // offline probes that check slot layout against the Python reference.
    const float* last_obs() const { return model_.input(); }

    // Re-anchor the heading alignment (called when a fresh playback
    // timeline starts, e.g. the first planner motion is adopted).
    void request_heading_reinit() { reinitialize_heading_ = true; }

private:
    // The reference root the heading aligns to comes from the planner motion
    // in g1/teleop and from the SMPL anchor in smpl mode.
    void update_heading_state(const MotionSequence50Hz& motion, int cursor,
                              const StateLogger& logger,
                              const SmplPoseWindow* smpl);
    void fill_obs(float* dst, const MotionSequence50Hz& motion, int cursor,
                  bool playing, const StateLogger& logger,
                  const VR3Point* vr3point, const SmplPoseWindow* smpl) const;
    // Anchor orientation (6D per frame) for the given per-frame reference
    // root quats; the motion overload samples them off the playback.
    void fill_anchor_orientation(float* out, const std::array<double, 4>* ref_roots,
                                 int num_frames,
                                 const std::array<double, 4>& base_quat) const;
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
