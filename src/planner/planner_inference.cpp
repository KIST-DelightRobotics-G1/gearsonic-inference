#include "planner/planner_inference.hpp"
#include "common/joint_order.hpp"
#include "common/math_utils.hpp"
#include "motion/input_handler.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>

namespace kist {

// ─── constants ────────────────────────────────────────────────────────────────

static constexpr double kLoopPeriodS = 0.1;   // 10 Hz planner thread

// Replan intervals per locomotion type (gear_sonic deploy defaults)
static constexpr double kReplanIntervalRunning  = 0.1;
static constexpr double kReplanIntervalCrawling = 0.2;
static constexpr double kReplanIntervalBoxing   = 1.0;
static constexpr double kReplanIntervalDefault  = 1.0;

PlannerInference& PlannerInference::instance() {
    static PlannerInference inst;
    return inst;
}

bool PlannerInference::start(const std::string& onnx_path, Precision precision,
                             double default_height, int64_t initial_random_seed) {
    default_height_      = default_height;
    current_random_seed_ = initial_random_seed;

    Options opts;
    opts.precision = precision;
    opts.deviceID  = 0;

    std::string trt_path;
    if (!ConvertONNXToTRT(opts, onnx_path, trt_path)) {
        std::cerr << "[PlannerInference] ConvertONNXToTRT failed\n";
        return false;
    }
    if (!engine_.Initialize(trt_path, 0)) {
        std::cerr << "[PlannerInference] engine Initialize failed\n";
        return false;
    }
    if (!engine_.InitInputs()) {
        std::cerr << "[PlannerInference] engine InitInputs failed\n";
        return false;
    }

    // Validate that the engine is actually the planner model we expect —
    // every tensor name used at runtime plus the load-bearing dims. Fails
    // here with a clear message instead of strangely at the first inference.
    // (gear_sonic validates the same names at construction.)
    if (!validate_engine())
        return false;

    if (cudaStreamCreate(&stream_) != cudaSuccess) {
        std::cerr << "[PlannerInference] cudaStreamCreate failed\n";
        return false;
    }

    // Pinned host buffers (sizes match the model's tensor shapes)
    context_.resize(kContextFrames * kQposDim);
    mode_.resize(1);
    target_vel_.resize(1);
    movement_direction_.resize(3);
    facing_direction_.resize(3);
    random_seed_.resize(1);
    height_.resize(1);
    has_specific_target_.resize(1);
    specific_target_positions_.resize(12);
    specific_target_headings_.resize(4);
    allowed_pred_num_tokens_.resize(11);
    mujoco_qpos_.resize(kMaxPlannerFrames * kQposDim);
    num_pred_frames_.resize(1);

    stop_        = false;
    loop_thread_ = std::thread(&PlannerInference::loop, this);
    std::cout << "[PlannerInference] started (10 Hz)\n";
    return true;
}

bool PlannerInference::validate_engine() {
    static const char* kInputs[] = {
        "context_mujoco_qpos", "mode", "target_vel", "movement_direction",
        "facing_direction", "random_seed", "height", "has_specific_target",
        "specific_target_positions", "specific_target_headings",
        "allowed_pred_num_tokens"};
    static const char* kOutputs[] = {"mujoco_qpos", "num_pred_frames"};

    auto tensor_dim = [this](const std::string& name) -> size_t {
        std::vector<int64_t> shape;
        if (!engine_.GetTensorShape(name, shape))
            return 0;
        size_t n = 1;
        for (auto d : shape)
            if (d > 0) n *= static_cast<size_t>(d);
        return n;
    };

    auto inputs  = engine_.GetInputTensorNames();
    auto outputs = engine_.GetOutputTensorNames();

    for (const char* name : kInputs) {
        if (std::find(inputs.begin(), inputs.end(), name) == inputs.end()) {
            std::cerr << "[PlannerInference] engine is missing input tensor \"" << name << "\"\n";
            return false;
        }
    }
    for (const char* name : kOutputs) {
        if (std::find(outputs.begin(), outputs.end(), name) == outputs.end()) {
            std::cerr << "[PlannerInference] engine is missing output tensor \"" << name << "\"\n";
            return false;
        }
    }

    size_t ctx = tensor_dim("context_mujoco_qpos");
    size_t out = tensor_dim("mujoco_qpos");
    if (ctx != kContextFrames * kQposDim || out != size_t(kMaxPlannerFrames) * kQposDim) {
        std::cerr << "[PlannerInference] unexpected tensor dims: context=" << ctx
                  << " (want " << kContextFrames * kQposDim << "), mujoco_qpos=" << out
                  << " (want " << kMaxPlannerFrames * kQposDim << ")\n";
        return false;
    }
    return true;
}

void PlannerInference::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
    if (stream_) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

// ─── input tensors (matches gear_sonic UpdateInputTensors) ────────────────────

void PlannerInference::set_input_tensors(int mode, float target_vel, float target_height,
                                         const std::array<float, 3>& movement_direction,
                                         const std::array<float, 3>& facing_direction) {
    mode_[0]              = mode;
    target_vel_[0]        = target_vel;
    height_[0]            = target_height;
    std::copy(movement_direction.begin(), movement_direction.end(), movement_direction_.begin());
    std::copy(facing_direction.begin(),   facing_direction.end(),   facing_direction_.begin());
    random_seed_[0]       = current_random_seed_;

    has_specific_target_[0] = 0;
    std::fill(specific_target_positions_.begin(), specific_target_positions_.end(), 0.0f);
    std::fill(specific_target_headings_.begin(),  specific_target_headings_.end(),  0.0f);

    // Token-count mask: allow 9-11 tokens only (gear_sonic TRT backend default)
    std::fill(allowed_pred_num_tokens_.begin(), allowed_pred_num_tokens_.end(), 0);
    allowed_pred_num_tokens_[3] = 1;
    allowed_pred_num_tokens_[4] = 1;
    allowed_pred_num_tokens_[5] = 1;
}

// ─── initial context (matches gear_sonic InitializeContext) ───────────────────

void PlannerInference::initialize_context(const std::array<double, MotionSequence50Hz::kNumJoints>& joint_positions) {
    for (int n = 0; n < kContextFrames; ++n) {
        float* dst = context_.data() + n * kQposDim;
        dst[0] = 0.0f;
        dst[1] = 0.0f;
        dst[2] = static_cast<float>(default_height_);
        dst[3] = 1.0f;                              // quat w (zero-yaw normalized frame)
        dst[4] = dst[5] = dst[6] = 0.0f;            // quat xyz
        // DDS motor order == MuJoCo qpos joint order: copy without remapping
        for (int j = 0; j < MotionSequence50Hz::kNumJoints; ++j)
            dst[7 + j] = static_cast<float>(joint_positions[j]);
    }
}

// ─── context sampling (matches gear_sonic UpdateContextFromMotion) ────────────

void PlannerInference::update_context_from_motion(const MotionSequence50Hz& motion) {
    double gen_time = double(gen_frame_) / 50.0;

    for (int n = 0; n < kContextFrames; ++n) {
        double t      = gen_time + double(n) / 30.0;
        double f_50hz = t * 50.0;
        int    f0     = static_cast<int>(std::floor(f_50hz));
        f0            = std::clamp(f0, 0, motion.timesteps - 1);
        int    f1     = std::min(f0 + 1, motion.timesteps - 1);
        double w1     = f_50hz - std::floor(f_50hz);
        double w0     = 1.0 - w1;

        const auto& a = motion.frames[f0];
        const auto& b = motion.frames[f1];
        auto q        = quat_slerp(a.quaternion, b.quaternion, w1);

        float* dst = context_.data() + n * kQposDim;
        for (int i = 0; i < 3; ++i)
            dst[i] = static_cast<float>(w0 * a.position[i] + w1 * b.position[i]);
        for (int i = 0; i < 4; ++i)
            dst[3 + i] = static_cast<float>(q[i]);
        // sequence joints are IsaacLab order; scatter back to MuJoCo qpos order
        for (int j = 0; j < MotionSequence50Hz::kNumJoints; ++j)
            dst[7 + mujoco_to_isaaclab[j]] = static_cast<float>(w0 * a.joints[j] + w1 * b.joints[j]);
    }
}

// ─── 30Hz → 50Hz resampler (matches gear_sonic ResampleGeneratedSequence50Hz) ─

bool PlannerInference::resample_30hz_to_50hz(const float* mujoco_qpos, int num_pred_frames) {
    if (num_pred_frames <= 0 || num_pred_frames > kMaxPlannerFrames) {
        std::cerr << "[PlannerInference] invalid num_pred_frames: " << num_pred_frames << "\n";
        return false;
    }
    if (std::any_of(mujoco_qpos, mujoco_qpos + num_pred_frames * kQposDim,
                    [](float x) { return std::isnan(x); })) {
        std::cerr << "[PlannerInference] mujoco_qpos output contains NaNs\n";
        return false;
    }

    double motion_seconds = double(num_pred_frames) / 30.0;
    int    n_50hz         = static_cast<int>(std::floor(motion_seconds * 50.0));
    n_50hz = std::min(n_50hz, MotionSequence50Hz::kCapacity);
    if (n_50hz < 2) {
        std::cerr << "[PlannerInference] generated sequence too short: " << n_50hz << " frames\n";
        return false;
    }
    motion_seq_.resize(n_50hz);

    for (int f_50hz = 0; f_50hz < n_50hz; ++f_50hz) {
        double t       = double(f_50hz) / 50.0;
        double f_30hz  = t * 30.0;
        int    f0      = static_cast<int>(std::floor(f_30hz));
        int    f1      = std::min(f0 + 1, num_pred_frames - 1);
        double w1      = f_30hz - f0;
        double w0      = 1.0 - w1;

        const float* p0 = mujoco_qpos + f0 * kQposDim;
        const float* p1 = mujoco_qpos + f1 * kQposDim;

        auto& out = motion_seq_.frames[f_50hz];

        // Position (linear)
        for (int i = 0; i < 3; ++i)
            out.position[i] = w0 * p0[i] + w1 * p1[i];

        // Quaternion (SLERP)
        std::array<double, 4> q0{p0[3], p0[4], p0[5], p0[6]};
        std::array<double, 4> q1{p1[3], p1[4], p1[5], p1[6]};
        out.quaternion = quat_slerp(q0, q1, w1);

        // Joints (linear), gathered from MuJoCo qpos order into IsaacLab order
        for (int j = 0; j < MotionSequence50Hz::kNumJoints; ++j)
            out.joints[j] = w0 * p0[7 + mujoco_to_isaaclab[j]] + w1 * p1[7 + mujoco_to_isaaclab[j]];
    }

    // Joint velocities: forward finite difference, last frame duplicated
    for (int f = 0; f < n_50hz - 1; ++f)
        for (int j = 0; j < MotionSequence50Hz::kNumJoints; ++j)
            motion_seq_.frames[f].joint_velocities[j] =
                (motion_seq_.frames[f + 1].joints[j] - motion_seq_.frames[f].joints[j]) * 50.0;
    motion_seq_.frames[n_50hz - 1].joint_velocities = motion_seq_.frames[n_50hz - 2].joint_velocities;

    return true;
}

// ─── inference + publish ──────────────────────────────────────────────────────

bool PlannerInference::run_and_publish() {
    engine_.SetInputDataAsync("context_mujoco_qpos",       context_,                   stream_);
    engine_.SetInputDataAsync("mode",                      mode_,                      stream_);
    engine_.SetInputDataAsync("target_vel",                target_vel_,                stream_);
    engine_.SetInputDataAsync("movement_direction",        movement_direction_,        stream_);
    engine_.SetInputDataAsync("facing_direction",          facing_direction_,          stream_);
    engine_.SetInputDataAsync("random_seed",               random_seed_,               stream_);
    engine_.SetInputDataAsync("height",                    height_,                    stream_);
    engine_.SetInputDataAsync("has_specific_target",       has_specific_target_,       stream_);
    engine_.SetInputDataAsync("specific_target_positions", specific_target_positions_, stream_);
    engine_.SetInputDataAsync("specific_target_headings",  specific_target_headings_,  stream_);
    engine_.SetInputDataAsync("allowed_pred_num_tokens",   allowed_pred_num_tokens_,   stream_);

    if (!engine_.Enqueue(stream_)) {
        std::cerr << "[PlannerInference] Enqueue failed\n";
        return false;
    }

    engine_.GetOutputDataAsync("mujoco_qpos",     mujoco_qpos_,     stream_);
    engine_.GetOutputDataAsync("num_pred_frames", num_pred_frames_, stream_);
    cudaStreamSynchronize(stream_);

    if (!resample_30hz_to_50hz(mujoco_qpos_.data(), num_pred_frames_[0]))
        return false;

    motion_seq_.gen_frame = gen_frame_;
    motion_50hz_buf.SetData(motion_seq_);
    last_publish_ = std::chrono::steady_clock::now();
    return true;
}

// ─── initialization (matches gear_sonic Initialize) ───────────────────────────

bool PlannerInference::initialize_once() {
    auto state = UnitreeStateReader::instance().unitree_state_buf.GetData();
    if (!state)
        return false;  // no robot state yet; retry next tick

    std::array<double, MotionSequence50Hz::kNumJoints> joint_positions{};
    for (int i = 0; i < MotionSequence50Hz::kNumJoints; ++i)
        joint_positions[i] = state->motors[i].q;

    initialize_context(joint_positions);
    set_input_tensors(static_cast<int>(LocomotionMode::IDLE),
                      -1.0f,                 // target_vel: mode default
                      -1.0f,                 // height: mode default
                      {0.0f, 0.0f, 0.0f},    // no movement
                      {1.0f, 0.0f, 0.0f});   // face forward (zero-yaw frame)
    gen_frame_ = 0;

    if (!run_and_publish())
        return false;

    // Baseline for replan-gating comparisons: same values as the initial plan
    last_movement_state_        = MovementState{};
    replan_interval_counter_    = 0.0;
    initialized_                = true;
    std::cout << "[PlannerInference] initialized (initial IDLE plan published)\n";
    return true;
}

// ─── replan gating (matches gear_sonic Planner() thread body) ─────────────────

bool PlannerInference::need_replan(const MovementState& ms) {
    const auto& last = last_movement_state_;
    bool mode_changed   = ms.locomotion_mode    != last.locomotion_mode;
    bool facing_changed = ms.facing_direction   != last.facing_direction;
    bool height_changed = ms.height             != last.height;
    bool speed_changed  = ms.movement_speed     != last.movement_speed;
    bool dir_changed    = ms.movement_direction != last.movement_direction;

    auto mode        = static_cast<LocomotionMode>(ms.locomotion_mode);
    bool under_static = is_static_motion_mode(mode);
    bool is_running   = mode == LocomotionMode::RUN;
    bool is_crawling  = mode == LocomotionMode::CRAWLING;
    bool is_boxing    = mode == LocomotionMode::LEFT_PUNCH ||
                        mode == LocomotionMode::RIGHT_PUNCH ||
                        mode == LocomotionMode::RANDOM_PUNCH ||
                        mode == LocomotionMode::LEFT_HOOK ||
                        mode == LocomotionMode::RIGHT_HOOK;

    double interval = is_running  ? kReplanIntervalRunning
                    : is_crawling ? kReplanIntervalCrawling
                    : is_boxing   ? kReplanIntervalBoxing
                                  : kReplanIntervalDefault;

    bool time_to_replan = false;
    if (replan_interval_counter_ >= interval) {
        replan_interval_counter_ = 0.0;
        time_to_replan = true;
    }

    if (mode_changed || facing_changed || height_changed)
        return true;
    return !under_static &&
           (speed_changed || dir_changed || (time_to_replan && ms.movement_speed != 0.0));
}

// ─── loop ─────────────────────────────────────────────────────────────────────

void PlannerInference::loop() try {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(static_cast<int>(kLoopPeriodS * 1e6));

    while (!stop_) {
        auto t0 = clock::now();

        // Reseed request (VLA -> origin recovery): drop the stale context
        // and go back through initialize_once. The buffer is cleared first
        // so consumers can treat any published motion as post-reseed.
        uint64_t reinit_req = reinit_requested_.load();
        if (reinit_req != reinit_done_.load() && initialized_) {
            std::cout << "[PlannerInference] reseed requested — reinitializing\n";
            motion_50hz_buf.Clear();
            initialized_ = false;
        }

        if (!initialized_) {
            if (initialize_once())
                reinit_done_.store(reinit_req);
            std::this_thread::sleep_until(t0 + period);
            continue;
        }

        replan_interval_counter_ += kLoopPeriodS;

        auto ms = InputHandler::instance().movement_buf.GetData();
        if (!ms) {
            std::this_thread::sleep_until(t0 + period);
            continue;
        }

        if (need_replan(*ms)) {
            last_movement_state_ = *ms;

            set_input_tensors(
                ms->locomotion_mode,
                static_cast<float>(ms->movement_speed),
                static_cast<float>(ms->height),
                {static_cast<float>(ms->movement_direction[0]),
                 static_cast<float>(ms->movement_direction[1]),
                 static_cast<float>(ms->movement_direction[2])},
                {static_cast<float>(ms->facing_direction[0]),
                 static_cast<float>(ms->facing_direction[1]),
                 static_cast<float>(ms->facing_direction[2])});

            // Context source: the control loop's blended motion at its actual
            // playback cursor when available (gear_sonic UpdatePlanning
            // semantics); otherwise our own last output + wall-clock estimate.
            MotionSequence50Hz snapshot;
            int  cursor        = 0;
            bool have_snapshot = false;
            {
                std::lock_guard<std::mutex> lock(provider_mutex_);
                if (playback_provider_)
                    have_snapshot = playback_provider_(snapshot, cursor);
            }
            if (have_snapshot) {
                gen_frame_ = cursor + kLookAheadSteps;
                update_context_from_motion(snapshot);
            } else {
                auto elapsed_s = std::chrono::duration<double>(t0 - last_publish_).count();
                gen_frame_ = static_cast<int>(elapsed_s * 50.0) + kLookAheadSteps;
                update_context_from_motion(motion_seq_);
            }
            run_and_publish();
        }

        std::this_thread::sleep_until(t0 + period);
    }
} catch (const std::exception& e) {
    std::cerr << "[PlannerInference] loop exception: " << e.what() << "\n";
} catch (...) {
    std::cerr << "[PlannerInference] loop unknown exception\n";
}

} // namespace kist
