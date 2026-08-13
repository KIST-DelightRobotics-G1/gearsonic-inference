#pragma once

#include "common/data_buffer.hpp"
#include "motion/movement_state.hpp"
#include "planner/motion_sequence_50hz.hpp"
#include "tensorrt/InferenceEngine.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace kist {

class PlannerInference {
public:
    // ── lifecycle ──────────────────────────────────────────────
    static PlannerInference& instance();
    bool             start(const std::string& onnx_path,
                           Precision precision = Precision::FP16,
                           double    default_height = 0.788740,
                           int64_t   initial_random_seed = 1234);
    void             stop();

    // ── output (read from any thread) ──────────────────────────
    // Each published sequence carries its own gen_frame, so the motion
    // and its blend point are always read together atomically.
    DataBuffer<MotionSequence50Hz> motion_50hz_buf;

    // ── playback provider (control loop -> planner context) ────
    // When set, replans sample their context from the consumer's blended
    // motion at its actual playback cursor (gear_sonic UpdatePlanning
    // semantics). Without it, falls back to the planner's own last output
    // and a wall-clock cursor estimate.
    using PlaybackProvider = std::function<bool(MotionSequence50Hz&, int&)>;
    void set_playback_provider(PlaybackProvider provider) {
        std::lock_guard<std::mutex> lock(provider_mutex_);
        playback_provider_ = std::move(provider);
    }

    // ── reseed (VLA -> origin recovery) ────────────────────────
    // Drop the (stale) planning context and re-run the startup
    // initialization: clear the published motion, reseed the context from
    // the measured joints, publish a fresh IDLE sequence. Returns a ticket;
    // the reseed is complete once reinit_done() >= that ticket. Any motion
    // published after completion is guaranteed post-reseed.
    uint64_t request_reinit() { return ++reinit_requested_; }
    uint64_t reinit_done() const { return reinit_done_.load(); }

private:
    PlannerInference() = default;

    void loop();
    bool validate_engine();
    bool initialize_once();
    bool need_replan(const MovementState& ms);
    void set_input_tensors(int mode, float target_vel, float target_height,
                           const std::array<float, 3>& movement_direction,
                           const std::array<float, 3>& facing_direction);
    bool run_and_publish();
    void initialize_context(const std::array<double, MotionSequence50Hz::kNumJoints>& joint_positions);
    void update_context_from_motion(const MotionSequence50Hz& motion);
    bool resample_30hz_to_50hz(const float* mujoco_qpos, int num_pred_frames);

    TRTInferenceEngine engine_;
    MotionSequence50Hz motion_seq_;
    cudaStream_t       stream_{nullptr};

    // ── persistent model input/output buffers ──────────────────
    // Pinned host memory + a dedicated stream, as the gear_sonic TRT
    // backend does (true async H2D/D2H copies). Sized in start().
    static constexpr int kContextFrames = 4;
    TPinnedVector<float>   context_;
    TPinnedVector<int64_t> mode_;
    TPinnedVector<float>   target_vel_;
    TPinnedVector<float>   movement_direction_;
    TPinnedVector<float>   facing_direction_;
    TPinnedVector<int64_t> random_seed_;
    TPinnedVector<float>   height_;
    TPinnedVector<int64_t> has_specific_target_;
    TPinnedVector<float>   specific_target_positions_;
    TPinnedVector<float>   specific_target_headings_;
    TPinnedVector<int64_t> allowed_pred_num_tokens_;
    TPinnedVector<float>   mujoco_qpos_;
    TPinnedVector<int32_t> num_pred_frames_;

    // ── planning state ──────────────────────────────────────────
    int           gen_frame_{0};   // blend point of the plan being generated
    bool          initialized_{false};
    std::atomic<uint64_t> reinit_requested_{0};
    std::atomic<uint64_t> reinit_done_{0};
    double        default_height_{0.788740};
    int64_t       current_random_seed_{1234};   // gear_sonic initial_random_seed
    MovementState last_movement_state_{};
    double        replan_interval_counter_{0.0};
    std::chrono::steady_clock::time_point last_publish_;

    static constexpr int kLookAheadSteps = 2;  // gear_sonic default (40 ms at 50Hz)

    std::mutex       provider_mutex_;
    PlaybackProvider playback_provider_;

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
