#pragma once

#include "common/data_buffer.hpp"
#include "pico/pico_vr_body_pose.hpp"
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
    bool calibrated() const    { return calibrated_; }

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
    DataBuffer<VR3Point> vr3point_buf;

private:
    TeleopTracker() = default;

    void loop();
    void check_calibration_gesture();
    void process(const PicoVRBodyPose& body);

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

    std::atomic<bool> calibrated_{false};
    std::atomic<bool> calibrate_request_{false};
    std::atomic<bool> reset_request_{false};

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};

    static constexpr double kLoopDt = 0.02;  // 50Hz, original POSE loop rate
    static constexpr double kTriggerIdle    = 0.5;  // gesture needs triggers released
    static constexpr int    kCalibHoldTicks = 50;   // 1s at 50Hz
};

} // namespace kist
