#include "control/whole_body_controller.hpp"
#include "common/joint_order.hpp"
#include "common/math_utils.hpp"
#include "common/robot_params.hpp"
#include "common/thread_priority.hpp"
#include "motion/input_handler.hpp"
#include "planner/planner_inference.hpp"
#include "teleop/teleop_tracker.hpp"
#include "unitree/unitree_state_reader.hpp"

#include "control/robot_ownership.hpp"
#include "vla/vla_initial_pose.hpp"
#include "vla/vla_token_receiver.hpp"

#include <algorithm>
#include <iostream>

namespace kist {

static constexpr double kMaxJointVel  = 35.0;   // rad/s, safety abort
static constexpr double kStateStaleMs = 500.0;  // LowState absence threshold

WholeBodyController& WholeBodyController::instance() {
    static WholeBodyController inst;
    return inst;
}

bool WholeBodyController::start(const std::string& encoder_path,
                                const std::string& decoder_path,
                                bool auto_start_control) {
    auto_start_ = auto_start_control;

    if (!encoder_.init(encoder_path))
        return false;
    if (!decoder_.init(decoder_path))
        return false;

    stop_        = false;
    loop_thread_ = std::thread(&WholeBodyController::loop, this);
    try_realtime_priority(loop_thread_, 80, "WholeBodyController");
    std::cout << "[WholeBodyController] started (50 Hz)\n";
    return true;
}

void WholeBodyController::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
    // The buffer's last word is always damping, so a still-running writer
    // can never keep republishing a stale pose target.
    publish_damping();
}

bool WholeBodyController::playback_snapshot(MotionSequence50Hz& motion, int& cursor) const {
    std::lock_guard<std::mutex> lock(playback_mutex_);
    if (playback_motion_.timesteps == 0)
        return false;
    motion = playback_motion_;
    cursor = cursor_;
    return true;
}

// ─── safety / robot state ─────────────────────────────────────────────────────

bool WholeBodyController::check_safety() {
    auto st = UnitreeStateReader::instance().unitree_state_buf.GetDataWithTime();
    if (!st.HasData() || st.GetAgeMs() > kStateStaleMs) {
        std::cerr << "[WholeBodyController] LowState missing/stale — stopping\n";
        return false;
    }
    return true;
}

// On any safety failure, overwrite the outgoing command with zero-torque
// damping so the (running) writer never keeps publishing a stale target.
void WholeBodyController::publish_damping() {
    MotorCommand damping;
    for (int i = 0; i < 29; ++i) {
        damping.kp[i] = 0.0f;
        damping.kd[i] = 8.0f;
    }
    motor_command_buf.SetData(damping);
}

bool WholeBodyController::gather_robot_state() {
    auto st = UnitreeStateReader::instance().unitree_state_buf.GetData();
    if (!st)
        return false;

    StateLogger::Entry e;
    e.base_quat    = {st->imu_pelvis.quaternion[0], st->imu_pelvis.quaternion[1],
                      st->imu_pelvis.quaternion[2], st->imu_pelvis.quaternion[3]};
    e.base_ang_vel = {st->imu_pelvis.gyroscope[0], st->imu_pelvis.gyroscope[1],
                      st->imu_pelvis.gyroscope[2]};
    for (int i = 0; i < 29; ++i) {
        int m = mujoco_to_isaaclab[i];
        e.body_q[i]  = st->motors[m].q - g1_default_angles[m];
        e.body_dq[i] = st->motors[m].dq;
        if (std::abs(e.body_dq[i]) > kMaxJointVel) {
            std::cerr << "[WholeBodyController] joint velocity " << e.body_dq[i]
                      << " rad/s over limit — stopping\n";
            return false;
        }
    }
    e.last_action = decoder_.last_action();
    logger_.Log(e);
    return true;
}

// ─── INIT: 3s linear ramp to the default standing pose ────────────────────────

void WholeBodyController::tick_init() {
    auto st = UnitreeStateReader::instance().unitree_state_buf.GetData();
    if (!st)
        return;

    // Interpolate from the *live measured* position each tick (gear_sonic
    // InitControl): the target stays near the actual pose all the way, so
    // tracking error — and the torque it commands — stays small.
    double ratio = std::clamp(double(init_ticks_) / kInitTicks, 0.0, 1.0);
    MotorCommand cmd;
    for (int i = 0; i < 29; ++i) {
        cmd.q_target[i] = static_cast<float>(st->motors[i].q * (1.0 - ratio) +
                                             g1_default_angles[i] * ratio);
        cmd.kp[i] = g1_kps[i];
        cmd.kd[i] = g1_kds[i];
    }
    motor_command_buf.SetData(cmd);

    if (++init_ticks_ >= kInitTicks) {
        state_ = State::WAIT_FOR_CONTROL;
        std::cout << "[WholeBodyController] INIT ramp done -> WAIT_FOR_CONTROL\n";
    }
}

// ─── playback advance + planner motion blending ───────────────────────────────
// gear_sonic CurrentFrameAdvancement: adopt/blend new planner sequences, then
// advance the cursor by one frame per tick, clamping at the end.

void WholeBodyController::advance_playback() {
    std::lock_guard<std::mutex> lock(playback_mutex_);

    auto plan = PlannerInference::instance().motion_50hz_buf.GetDataWithTime();
    if (plan.HasData() && plan.timestamp != last_plan_stamp_) {
        last_plan_stamp_ = plan.timestamp;
        const auto& gen = *plan.data;

        if (playback_motion_.timesteps == 0) {
            // first planner motion: copy wholesale, restart playback
            playback_motion_ = gen;
            cursor_  = 0;
            playing_ = true;
            encoder_.request_heading_reinit();
            std::cout << "[WholeBodyController] first planner motion adopted ("
                      << gen.timesteps << " frames)\n";
        } else {
            int fgen = gen.gen_frame;
            int new_len = fgen - cursor_ + gen.timesteps;
            if (new_len > 0) {
                new_len = std::min(new_len, MotionSequence50Hz::kCapacity);
                int blend_start = std::max(0, fgen - cursor_);

                MotionSequence50Hz out;
                out.resize(new_len);
                for (int f = 0; f < new_len; ++f) {
                    int f_old = std::clamp(f + cursor_, 0, playback_motion_.timesteps - 1);
                    int f_new = std::clamp(f + cursor_ - fgen, 0, gen.timesteps - 1);
                    double w_new = std::clamp(double(f - blend_start) / kBlendFrames, 0.0, 1.0);
                    double w_old = 1.0 - w_new;

                    const auto& a = playback_motion_.frames[f_old];
                    const auto& b = gen.frames[f_new];
                    auto& o = out.frames[f];
                    for (int j = 0; j < 29; ++j) {
                        o.joints[j]           = w_old * a.joints[j] + w_new * b.joints[j];
                        o.joint_velocities[j] = w_old * a.joint_velocities[j] + w_new * b.joint_velocities[j];
                    }
                    for (int i = 0; i < 3; ++i)
                        o.position[i] = w_old * a.position[i] + w_new * b.position[i];
                    o.quaternion = quat_slerp(a.quaternion, b.quaternion, w_new);
                }
                playback_motion_ = std::move(out);
                cursor_ = 0;
            }
        }
    }

    // advance one frame per tick, hold last frame at the end
    if (playing_ && playback_motion_.timesteps > 0)
        cursor_ = std::min(cursor_ + 1, playback_motion_.timesteps - 1);
}

// ─── CONTROL tick ─────────────────────────────────────────────────────────────

void WholeBodyController::tick_control() {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    if (!gather_robot_state()) {
        publish_damping();
        state_ = State::WAIT_FOR_CONTROL;
        return;
    }

    // A live external token stream (kist-vla-inference) overrides the
    // planner+encoder path entirely.
    if (tick_vla_control(t0))
        return;

    // Snapshot the playback state. The lock protects cursor_ against the
    // planner thread's playback_snapshot(); playback_motion_ itself is only
    // ever written by this thread (advance_playback), so reading it through
    // the pointer after the lock is released is safe.
    MotionSequence50Hz* motion = nullptr;
    int cursor = 0;
    {
        std::lock_guard<std::mutex> lock(playback_mutex_);
        motion = &playback_motion_;
        cursor = cursor_;
    }
    if (motion->timesteps == 0) {
        // no planner motion yet — nothing to track
        advance_playback();
        return;
    }

    // Teleop upper-body target: buffer presence selects the encoder mode
    // (empty until calibrated, cleared when body tracking drops -> g1).
    auto vr3 = TeleopTracker::instance().vr3point_buf.GetData();

    TokenEncoder::Token token;
    if (!encoder_.step(*motion, cursor, playing_, logger_, vr3.get(), token))
        return;
    auto t1 = clock::now();

    MotorCommand cmd;
    if (!decoder_.step(token, logger_, cmd))
        return;
    auto t2 = clock::now();

    motor_command_buf.SetData(cmd);
    advance_playback();

    {
        std::lock_guard<std::mutex> l(timing_mutex_);
        timing_.encoder = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        timing_.decoder = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        timing_.total   = std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - t0).count();
    }
}

// ─── loop ─────────────────────────────────────────────────────────────────────

void WholeBodyController::loop() try {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(static_cast<int>(kControlDt * 1e6));

    while (!stop_) {
        auto t0 = clock::now();

        // First-come ownership: teleop claims the robot while calibrated
        // and releases when calibration drops. (VLA claims/latches inside
        // tick_vla_control; the e-stop below outranks both.)
        {
            auto& ownership = RobotOwnership::instance();
            if (TeleopTracker::instance().calibrated())
                ownership.try_claim(RobotOwnership::Owner::kTeleop);
            else
                ownership.release(RobotOwnership::Owner::kTeleop);
        }

        // Operator e-stop: terminal state — damping every tick, no recovery.
        if (InputHandler::instance().estop()) {
            if (state_ != State::ESTOP) {
                std::cerr << "[WholeBodyController] EMERGENCY STOP — damping, control terminated\n";
                state_ = State::ESTOP;
            }
            publish_damping();
            std::this_thread::sleep_until(t0 + period);
            continue;
        }

        if (!check_safety()) {
            publish_damping();
            std::this_thread::sleep_until(t0 + period);
            continue;
        }

        switch (state_) {
            case State::INIT:
                tick_init();
                break;
            case State::WAIT_FOR_CONTROL:
                if (operator_start_ || auto_start_) {
                    state_ = State::CONTROL;
                    std::cout << "[WholeBodyController] -> CONTROL\n";
                }
                break;
            case State::CONTROL:
                tick_control();
                break;
            case State::ESTOP:
                break;  // unreachable — handled by the gate above
        }

        std::this_thread::sleep_until(t0 + period);
    }
} catch (const std::exception& e) {
    std::cerr << "[WholeBodyController] loop exception: " << e.what() << "\n";
}

// ─── VLA external-token mode ──────────────────────────────────────────────────

bool WholeBodyController::tick_vla_control(std::chrono::steady_clock::time_point t0) {
    using clock = std::chrono::steady_clock;
    auto vla = VlaTokenReceiver::instance().latent_buf.GetDataWithTime();

    if (!vla_active_) {
        if (!vla.HasData() || vla.GetAgeMs() > kVlaTokenFreshMs)
            return false;  // no live stream — planner path as usual

        // First-come ownership: if teleop already holds the robot, ignore
        // the token stream entirely (log once per denied stream).
        if (!RobotOwnership::instance().try_claim(RobotOwnership::Owner::kVla)) {
            if (!vla_denied_logged_) {
                std::cout << "[WholeBodyController] VLA tokens arriving but teleop "
                             "owns the robot — ignoring VLA until restart\n";
                vla_denied_logged_ = true;
            }
            return false;
        }
        std::cout << "[WholeBodyController] VLA token stream live -> external token mode\n";
        vla_active_ = true;
    }

    // Stream-loss handling: losing the token stream ends the VLA session
    // (latched — a resumed stream may not silently retake the robot, and
    // the planner path's playback timeline froze at the pre-VLA state, so
    // falling back there would yank the robot toward a stale reference).
    // But the decoder and robot state are still healthy, so instead of
    // damping (a fall when not hung) we blend the last commanded token to
    // the verified safe standing token and keep balancing there: the robot
    // ends up standing in place, waiting. E-stop still gives damping;
    // restart is the only way back to control.
    TokenEncoder::Token token;
    if (!vla_stream_lost_ && (!vla.HasData() || vla.GetAgeMs() > kVlaTokenHoldMs)) {
        std::cerr << "[WholeBodyController] VLA token stream LOST (>"
                  << kVlaTokenHoldMs << "ms stale) — blending to safe "
                     "standing hold (e-stop to stop, restart to recover)\n";
        vla_stream_lost_ = true;
        vla_loss_blend_tick_ = 0;
    }
    if (vla_stream_lost_) {
        float alpha = 1.0f;
        if (vla_loss_blend_tick_ < kVlaLossBlendTicks) {
            alpha = static_cast<float>(++vla_loss_blend_tick_) / kVlaLossBlendTicks;
        }
        for (size_t i = 0; i < token.size(); ++i)
            token[i] = (1.0f - alpha) * last_vla_token_[i]
                       + alpha * kVlaSafeStandingToken[i];
    } else {
        // VlaLatentAction::token and TokenEncoder::Token are both
        // std::array<float, 64>.
        token = vla.data->token;
        last_vla_token_ = token;
    }

    MotorCommand cmd;
    if (!decoder_.step(token, logger_, cmd))
        return true;
    motor_command_buf.SetData(cmd);

    {
        std::lock_guard<std::mutex> l(timing_mutex_);
        timing_.encoder = 0;
        timing_.decoder =
            std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - t0).count();
        timing_.total = timing_.decoder;
    }
    return true;
}

} // namespace kist
