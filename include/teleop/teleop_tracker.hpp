#pragma once

#include "common/data_buffer.hpp"
#include "pico/pico_vr_body_pose.hpp"
#include "teleop/smpl_arm_reach.hpp"
#include "teleop/smpl_pose_window.hpp"
#include "teleop/vr_3point.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

namespace kist {

// Upper-body teleop tracker, ported from gear_sonic's ThreePointPose
// (pico_manager_thread_server.py): consumes the PICO SMPL body pose and
// produces the calibrated 3-point target (L wrist, R wrist, neck) that
// drives the encoder's teleop mode.
//
// Pipeline per body sample:
//   1. Unity frame -> robot frame (Q = [-x, z, y] conjugation)
//   2. per-keypoint orientation offsets (root/wrists/neck)
//   3. root(pelvis)-relative positions + orientations
//   4. calibration: neck-inverse rotation + wrist offsets captured against
//      the G1 zero-pose FK wrist poses; neck position from a fixed
//      kinematic chain (root -> torso +Z -> neck along its local Z)
//
// vr3point_buf is only published while calibrated — cleared on reset /
// teleop off; consumers key off "has data" to gate teleop. A body-stream
// dropout does NOT clear it: the reader holds the last body sample, so the
// last teleop target stays published and the arms freeze in place until
// the stream resumes.
//
// Full-body flavor (A held 1s, from IDLE): the whole 24-joint body goes
// through smpl_fk and ten 50Hz frames are published as an SmplPoseWindow
// (encoder smpl mode, 2). The two flavors are exclusive — whichever engages
// first owns teleop, the other gesture is ignored until it lets go — and
// either counts as "teleop engaged" for the arbiter (calibrated()). Unlike
// the upper-body flavor, a body-stream dropout here DISENGAGES after
// kFullBodyStaleMs: the legs are referenced too, so freezing mid-step is
// not a safe hold — the arbiter then returns the robot to the origin.
class TeleopTracker {
public:
    static TeleopTracker& instance();
    void start();
    void stop();

    // Capture calibration on the next body sample. The operator must hold
    // the reference pose: upright, upper arms down, forearms bent 90°
    // forward, palms inward, looking straight ahead (= robot zero pose).
    //
    // Controller gesture (B held 1s, triggers released) toggles: not
    // calibrated -> calibrate (teleop on); calibrated -> teleop off
    // (full reset). Calibration is stateless: every engage captures neck
    // + wrists fresh, in the full reference pose (forearms 90° forward,
    // palms inward, looking straight ahead) — a bad capture is cured by
    // toggling off and on, and nothing stale survives a cycle.
    void request_calibration() { calibrate_request_ = true; }
    void reset_calibration()   { reset_request_ = true; }
    // Teleop engaged — either flavor. This is the arbiter's "operator holds
    // the robot" input; full-body and upper-body both claim through it.
    bool calibrated() const    { return calibrated_ || fullbody_engaged_; }
    bool fullbody() const      { return fullbody_engaged_; }

    // OPT-IN: measured robot joints (MuJoCo/DDS order) as the wrist
    // calibration reference (gear_sonic recalibrate_for_vr3pt) — maps the
    // operator's engage-moment wrists to the robot's current wrists (FK),
    // jump-free. Only accurate if the operator matches the robot's actual
    // arm pose at engage; with the standard reference pose the default
    // zero-pose constants are the anatomically correct anchor (hardware-
    // verified). Set before start().
    using MeasuredQProvider = std::function<bool(std::array<double, 29>&)>;
    void set_measured_q_provider(MeasuredQProvider p) { measured_q_provider_ = std::move(p); }

    // ── output ──────────────────────────────────────────────────
    DataBuffer<VR3Point>       vr3point_buf;     // upper-body flavor (mode 1)
    DataBuffer<SmplPoseWindow> smpl_window_buf;  // full-body flavor  (mode 2)

private:
    TeleopTracker() = default;

    void loop();
    void check_calibration_gesture();
    void process(const PicoVRBodyPose& body);

    // full-body flavor
    void check_fullbody_gesture();
    void process_fullbody(const PicoVRBodyPose& body);
    void disengage_fullbody(const char* why);

    // raw 3-point [L, R, neck], root-relative (before calibration)
    struct Raw3Point {
        std::array<std::array<double, 3>, 3> pos;
        std::array<std::array<double, 4>, 3> quat;  // wxyz
    };
    Raw3Point extract_raw(const PicoVRBodyPose& body) const;
    void capture_calibration(const Raw3Point& raw);
    void apply_calibration(const Raw3Point& raw, VR3Point& out) const;

    // Calibration state (tracker thread only)
    bool have_neck_calib_{false};
    std::array<double, 4> neck_quat_inv_{};
    std::array<double, 3> lwrist_pos_offset_{}, rwrist_pos_offset_{};
    std::array<double, 4> lwrist_rot_offset_{}, rwrist_rot_offset_{};

    MeasuredQProvider measured_q_provider_;

    // Body samples carry no device timestamp; dedup by buffer receive time.
    std::chrono::steady_clock::time_point last_body_time_{};

    // B-held calibration gesture (tracker thread only)
    int  calib_hold_ticks_{0};
    bool calib_gesture_latched_{false};

    // A-held full-body gesture + delay line (tracker thread only). The
    // encoder wants ten frames "ahead of the cursor"; live tracking has
    // none, so the policy tracks the operator kLookaheadTicks late and the
    // newest sample is held into the remaining slots (the window gear_sonic's
    // streamed-motion merger yields in steady state).
    int  fb_hold_ticks_{0};
    bool fb_gesture_latched_{false};
    static constexpr int kHistory = 16;
    std::array<SmplPose, kHistory> history_{};
    int hist_count_{0};
    int hist_head_{0};   // index of the newest frame
    std::atomic<bool> fullbody_engaged_{false};
    // Bring-up telemetry (1 Hz while engaged): SMPL-FK arm extension vs the
    // tracked wrist distance, to tell "the operator's arm rotations don't
    // encode the extension" from a mapping bug. Turn off once settled.
    static constexpr bool kFullBodyDebugLog = false;
    int fb_debug_ticks_{0};
    // Arms from tracked wrist positions instead of the headset's arm-joint
    // rotations (see smpl_arm_reach.hpp). Reach estimate resets on engage.
    static constexpr bool kFullBodyPositionArms = true;
    ArmReachState arm_reach_{};

    std::atomic<bool> calibrated_{false};
    std::atomic<bool> calibrate_request_{false};
    std::atomic<bool> reset_request_{false};

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};

    static constexpr double kLoopDt = 0.02;  // 50Hz, original POSE loop rate
    static constexpr double kTriggerIdle    = 0.5;  // gesture needs triggers released
    static constexpr int    kCalibHoldTicks = 50;   // 1s at 50Hz
    // Full-body: frames the policy lags the operator (latency knob, ticks;
    // upstream steady state is about one 5-frame chunk) and the body-stream
    // age past which the flavor disengages instead of freezing the legs.
    static constexpr int    kLookaheadTicks  = 5;
    static constexpr double kFullBodyStaleMs = 500.0;
};

} // namespace kist
