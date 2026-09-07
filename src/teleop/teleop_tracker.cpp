#include "teleop/teleop_tracker.hpp"
#include "common/math_utils.hpp"
#include "control/control_arbiter.hpp"
#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"
#include "teleop/g1_arm_fk.hpp"
#include "teleop/smpl_fk.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace kist {

namespace {

// Unity (X-right, Y-up, Z-forward) -> robot (X-forward, Y-left, Z-up):
// position [x,y,z] -> [-x, z, y]. The same permutation as a rotation
// matrix Q = [[-1,0,0],[0,0,1],[0,1,0]] is proper (det=+1), a 180° turn
// about (0,1,1)/sqrt(2) — so orientations map by quaternion conjugation.
const std::array<double, 4> kUnityToRobot = {0.0, 0.0, std::sqrt(0.5), std::sqrt(0.5)};

// Extrinsic xyz Euler (scipy "xyz") -> quaternion: R = Rz(c)·Ry(b)·Rx(a)
std::array<double, 4> euler_xyz_extrinsic(double a, double b, double c) {
    return quat_mul(quat_from_angle_axis(c, {0.0, 0.0, 1.0}),
           quat_mul(quat_from_angle_axis(b, {0.0, 1.0, 0.0}),
                    quat_from_angle_axis(a, {1.0, 0.0, 0.0})));
}

constexpr double kDeg = M_PI / 180.0;

// Per-keypoint frame corrections (gear_sonic OFFSETS), post-multiplied:
// [root, L wrist, R wrist, neck]
const std::array<std::array<double, 4>, 4> kOffsets = {
    euler_xyz_extrinsic(0.0,       0.0, -90.0 * kDeg),   // root: yaw -90°
    euler_xyz_extrinsic(90.0 * kDeg,  0.0, 0.0),          // L wrist: roll +90°
    euler_xyz_extrinsic(-90.0 * kDeg, 0.0, 180.0 * kDeg), // R wrist: roll -90°, yaw 180°
    euler_xyz_extrinsic(0.0,       0.0, -90.0 * kDeg),   // neck: yaw -90°
};

// G1 key-frame poses at the all-zero joint pose (root frame): FK through
// the URDF origin chain to {left,right}_wrist_yaw_link, plus the local
// offsets [0.18, ∓0.025, 0] from gear_sonic force.yaml. The calibration
// reference the operator's zero pose is matched against.
constexpr std::array<double, 3> kG1LWristPos = {0.3797694914, 0.1236272185, 0.0952214409};
constexpr std::array<double, 3> kG1RWristPos = {0.3797694914, -0.1236172185, 0.0952214409};
const std::array<double, 4> kG1LWristQuat = {0.9999999946, 0.0000300026, 0.0000274716, -0.0000957958};
const std::array<double, 4> kG1RWristQuat = {0.9999999946, -0.0000300026, 0.0000274716, 0.0000957958};

// Neck position kinematic chain (gear_sonic ThreePointPose)
constexpr double kTorsoLinkOffsetZ = 0.05;  // m, root -> torso_link
constexpr double kNeckLinkLength   = 0.35;  // m, torso_link -> neck along neck local Z

} // namespace

TeleopTracker& TeleopTracker::instance() {
    static TeleopTracker inst;
    return inst;
}

void TeleopTracker::start() {
    stop_        = false;
    loop_thread_ = std::thread(&TeleopTracker::loop, this);
    std::cout << "[TeleopTracker] started (50 Hz)\n";
}

void TeleopTracker::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
    vr3point_buf.Clear();
    smpl_window_buf.Clear();
}

// ─── loop ─────────────────────────────────────────────────────────────────────

void TeleopTracker::loop() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(static_cast<int>(kLoopDt * 1e6));

    while (!stop_) {
        auto t0 = clock::now();

        if (reset_request_.exchange(false)) {
            have_neck_calib_ = false;
            calibrated_      = false;
            vr3point_buf.Clear();
            if (fullbody_engaged_)
                disengage_fullbody("reset");
            std::cout << "[TeleopTracker] calibration reset\n";
        }

        check_calibration_gesture();
        check_fullbody_gesture();

        auto body = PicoVRReader::instance().body_buf.GetDataWithTime();
        if (!body.HasData()) {
            // no body sample yet this session (the reader holds the last
            // one through dropouts, so this only fires before the first)
            vr3point_buf.Clear();
        } else if (body.timestamp != last_body_time_) {
            last_body_time_ = body.timestamp;
            process(*body.data);
        }

        // Full-body runs every tick on the latest sample (sample-and-hold),
        // not per new sample: the window's frames must be 50Hz-spaced.
        if (fullbody_engaged_) {
            if (!body.HasData() || body.GetAgeMs() > kFullBodyStaleMs)
                disengage_fullbody("body stream stale");
            else
                process_fullbody(*body.data);
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

// B held 1s (triggers released — a pressed trigger means the operator is
// closing the Dex3 fingers, not gesturing) — a toggle: not calibrated ->
// calibrate (teleop on); calibrated -> reset (back to g1). Must be
// released before it can fire again; re-entry recalibrates fresh.
//
// Also gated by "B alone among the face buttons": A+B+X+Y is the e-stop
// combo, and any 2-button hold voids per-button gestures (see
// InputHandler). Without this the e-stop windup would flip teleop at
// exactly the wrong moment.
void TeleopTracker::check_calibration_gesture() {
    // While VLA holds the robot (or the origin recovery is running) B is
    // ignored outright — no hidden armed teleop can form, so the VLA exit
    // path always lands at the origin (see control_arbiter.hpp).
    auto arb_mode = ControlArbiter::instance().mode();
    if (arb_mode == ControlArbiter::Mode::kVla ||
        arb_mode == ControlArbiter::Mode::kRecovering ||
        fullbody_engaged_) {   // full-body owns teleop: B ignored until it lets go
        calib_hold_ticks_      = 0;
        calib_gesture_latched_ = false;
        return;
    }

    auto ctrl = PicoVRReader::instance().ctrl_buf.GetData();
    bool b_alone = ctrl && ctrl->btn_b &&
                   !ctrl->btn_a && !ctrl->btn_x && !ctrl->btn_y;
    bool held = b_alone &&
                ctrl->left_trigger < kTriggerIdle &&
                ctrl->right_trigger < kTriggerIdle;
    if (!held) {
        calib_hold_ticks_     = 0;
        calib_gesture_latched_ = false;
        return;
    }
    if (calib_gesture_latched_)
        return;
    if (++calib_hold_ticks_ >= kCalibHoldTicks) {
        calib_gesture_latched_ = true;
        if (calibrated_) {
            // teleop off: full reset — calibration is stateless across
            // engages. Every engage recaptures neck + wrists fresh in the
            // reference pose, so a bad or stale capture never outlives
            // one on/off cycle.
            have_neck_calib_ = false;
            calibrated_      = false;
            vr3point_buf.Clear();
            std::cout << "[TeleopTracker] gesture (B 1s): teleop off -> g1\n";
        } else {
            request_calibration();
            std::cout << "[TeleopTracker] gesture (B 1s): calibrating -> teleop on\n";
            if (!PicoVRReader::instance().body_buf.GetData())
                std::cerr << "[TeleopTracker] WARNING: no body tracking data — "
                             "calibration is pending until it arrives "
                             "(enable body tracking on the headset)\n";
        }
    }
}

// A held 1s (alone among the face buttons, triggers released) toggles the
// full-body flavor. Engages only from locomotion IDLE — the smpl encoder
// mode has no lower-body command slot, so walking cannot coexist with it
// (InputHandler's A tap has already dropped the mode to IDLE by the time
// the hold latches) — and only while the upper-body flavor is off: the
// flavors are exclusive, first engaged wins. Same VLA/recovery gate as B.
void TeleopTracker::check_fullbody_gesture() {
    auto arb_mode = ControlArbiter::instance().mode();
    if (arb_mode == ControlArbiter::Mode::kVla ||
        arb_mode == ControlArbiter::Mode::kRecovering ||
        calibrated_) {   // upper-body owns teleop: A ignored until it lets go
        fb_hold_ticks_      = 0;
        fb_gesture_latched_ = false;
        return;
    }

    auto ctrl = PicoVRReader::instance().ctrl_buf.GetData();
    bool a_alone = ctrl && ctrl->btn_a &&
                   !ctrl->btn_b && !ctrl->btn_x && !ctrl->btn_y;
    bool held = a_alone &&
                ctrl->left_trigger < kTriggerIdle &&
                ctrl->right_trigger < kTriggerIdle;
    if (!held) {
        fb_hold_ticks_      = 0;
        fb_gesture_latched_ = false;
        return;
    }
    if (fb_gesture_latched_)
        return;
    if (++fb_hold_ticks_ >= kCalibHoldTicks) {
        fb_gesture_latched_ = true;
        if (fullbody_engaged_) {
            disengage_fullbody("gesture (A 1s)");
        } else if (InputHandler::instance().mode() != static_cast<int>(LocomotionMode::IDLE)) {
            std::cout << "[TeleopTracker] gesture (A 1s) ignored: locomotion not IDLE\n";
        } else if (!PicoVRReader::instance().body_buf.GetData()) {
            std::cerr << "[TeleopTracker] gesture (A 1s) ignored: no body tracking data "
                         "(enable body tracking on the headset)\n";
        } else {
            hist_count_       = 0;
            arm_reach_        = ArmReachState{};
            fullbody_engaged_ = true;
            std::cout << "[TeleopTracker] gesture (A 1s): full-body teleop on (smpl)\n";
        }
    }
}

void TeleopTracker::disengage_fullbody(const char* why) {
    fullbody_engaged_ = false;
    hist_count_       = 0;
    smpl_window_buf.Clear();
    std::cout << "[TeleopTracker] full-body teleop off (" << why << ") -> g1\n";
}

// One SmplPose per tick into the delay line, then the window the encoder
// reads: frame f is the sample kLookaheadTicks - f ticks old, clamped to
// the newest (held) beyond it and to the oldest we have during warm-up.
void TeleopTracker::process_fullbody(const PicoVRBodyPose& body) {
    hist_head_ = (hist_count_ == 0) ? 0 : (hist_head_ + 1) % kHistory;
    history_[hist_head_] = smpl_fk(body);
    if (kFullBodyPositionArms)
        smpl_arms_from_tracked_wrists(history_[hist_head_], body, arm_reach_);
    if (hist_count_ < kHistory) ++hist_count_;

    if (kFullBodyDebugLog && ++fb_debug_ticks_ >= 50) {
        fb_debug_ticks_ = 0;
        // SMPL-24: shoulders 16/17, elbows 18/19, wrists 20/21; the tracked
        // stream's "wrist" keypoints are 22/23 (as the 3-point path uses).
        const auto& jl = history_[hist_head_].joints_local;
        auto dist = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
            return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
        };
        auto elbow_deg = [&](int s, int e, int w) {
            std::array<double, 3> u = {jl[s][0]-jl[e][0], jl[s][1]-jl[e][1], jl[s][2]-jl[e][2]};
            std::array<double, 3> v = {jl[w][0]-jl[e][0], jl[w][1]-jl[e][1], jl[w][2]-jl[e][2]};
            double nu = std::sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]), nv = std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
            double c = (u[0]*v[0]+u[1]*v[1]+u[2]*v[2]) / std::max(nu*nv, 1e-9);
            return std::acos(std::clamp(c, -1.0, 1.0)) * 180.0 / M_PI;
        };
        auto tracked = [&](int s, int w) {
            const auto& a = body.joints[s]; const auto& b = body.joints[w];
            return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
        };
        std::printf("[TeleopTracker] smpl arms  L: elbow %5.1f deg, shoulder->wrist %.2fm (tracked %.2fm)"
                    "   R: elbow %5.1f deg, %.2fm (tracked %.2fm)   [straight arm: ~180 deg, smpl ~0.51m]\n",
                    elbow_deg(16, 18, 20), dist(jl[16], jl[20]), tracked(16, 22),
                    elbow_deg(17, 19, 21), dist(jl[17], jl[21]), tracked(17, 23));
    }

    SmplPoseWindow w;
    for (int f = 0; f < SmplPoseWindow::kFrames; ++f) {
        int depth = kLookaheadTicks - f;           // ticks behind the newest
        depth = std::max(0, std::min(depth, hist_count_ - 1));
        w.frames[f] = history_[(hist_head_ - depth + kHistory) % kHistory];
    }
    smpl_window_buf.SetData(std::move(w));
}

void TeleopTracker::process(const PicoVRBodyPose& body) {
    Raw3Point raw = extract_raw(body);

    if (calibrate_request_.exchange(false))
        capture_calibration(raw);

    if (!calibrated_)
        return;

    VR3Point out;
    apply_calibration(raw, out);
    vr3point_buf.SetData(out);
}

// ─── SMPL -> raw root-relative 3-point ────────────────────────────────────────

TeleopTracker::Raw3Point TeleopTracker::extract_raw(const PicoVRBodyPose& body) const {
    // keypoints: root/pelvis(0), L wrist(22), R wrist(23), neck(12) —
    // neck instead of head(15): rigidly coupled to the torso, stabler.
    static constexpr int kSmplIdx[4] = {0, 22, 23, 12};

    std::array<std::array<double, 3>, 4> pos;
    std::array<std::array<double, 4>, 4> quat;
    for (int k = 0; k < 4; ++k) {
        const auto& j = body.joints[kSmplIdx[k]];  // [x,y,z, qx,qy,qz,qw]
        pos[k] = {-j[0], j[2], j[1]};
        std::array<double, 4> q = {j[6], j[3], j[4], j[5]};
        q = quat_mul(kUnityToRobot, quat_mul(q, quat_conjugate(kUnityToRobot)));
        quat[k] = quat_mul(q, kOffsets[k]);
    }

    Raw3Point raw;
    auto root_inv = quat_conjugate(quat[0]);
    for (int k = 1; k < 4; ++k) {
        std::array<double, 3> d = {pos[k][0] - pos[0][0],
                                   pos[k][1] - pos[0][1],
                                   pos[k][2] - pos[0][2]};
        raw.pos[k - 1]  = quat_rotate(root_inv, d);
        raw.quat[k - 1] = quat_mul(root_inv, quat[k]);
    }
    return raw;
}

// ─── calibration ──────────────────────────────────────────────────────────────

void TeleopTracker::capture_calibration(const Raw3Point& raw) {
    // Neck orientation: keep an existing capture (recalibration must not
    // jump from SMPL noise), otherwise store inv(current neck).
    if (!have_neck_calib_) {
        neck_quat_inv_   = quat_conjugate(raw.quat[2]);
        have_neck_calib_ = true;
    }

    // Wrist reference: zero-pose FK constants by default — the zero pose
    // has horizontal forearms, exactly the operator's reference pose, so
    // the anchors correspond anatomically. The measured-q reference
    // (gear_sonic recalibrate_for_vr3pt) is opt-in via the provider: it is
    // only accurate when the operator matches the robot's *actual* pose at
    // engage (default stance = forearms 34° down), which proved impractical
    // on hardware — anchoring horizontal to sagged forearms skews every
    // target ~0.2m low and the arms strain at the workspace edge.
    std::array<double, 3> ref_l_pos = kG1LWristPos, ref_r_pos = kG1RWristPos;
    std::array<double, 4> ref_l_rot = kG1LWristQuat, ref_r_rot = kG1RWristQuat;
    std::array<double, 29> q;
    if (measured_q_provider_) {
        if (measured_q_provider_(q)) {
            auto fk_l = g1_wrist_fk(q, true);
            auto fk_r = g1_wrist_fk(q, false);
            ref_l_pos = fk_l.position;   ref_l_rot = fk_l.quaternion;
            ref_r_pos = fk_r.position;   ref_r_rot = fk_r.quaternion;
            std::cout << "[TeleopTracker] wrist reference: measured-q FK\n";
        } else {
            std::cout << "[TeleopTracker] WARNING: measured-q provider gave no "
                         "joints — wrist reference falls back to the zero pose\n";
        }
    } else {
        std::cout << "[TeleopTracker] wrist reference: zero pose\n";
    }

    // Wrist offsets against the reference, in the neck-corrected frame.
    auto lw_pos = quat_rotate(neck_quat_inv_, raw.pos[0]);
    auto rw_pos = quat_rotate(neck_quat_inv_, raw.pos[1]);
    auto lw_rot = quat_mul(neck_quat_inv_, raw.quat[0]);
    auto rw_rot = quat_mul(neck_quat_inv_, raw.quat[1]);

    for (int i = 0; i < 3; ++i) {
        lwrist_pos_offset_[i] = lw_pos[i] - ref_l_pos[i];
        rwrist_pos_offset_[i] = rw_pos[i] - ref_r_pos[i];
    }
    lwrist_rot_offset_ = quat_mul(ref_l_rot, quat_conjugate(lw_rot));
    rwrist_rot_offset_ = quat_mul(ref_r_rot, quat_conjugate(rw_rot));

    calibrated_ = true;
    std::cout << "[TeleopTracker] calibration captured"
              << "  L offset [" << lwrist_pos_offset_[0] << ", " << lwrist_pos_offset_[1]
              << ", " << lwrist_pos_offset_[2] << "]"
              << "  R offset [" << rwrist_pos_offset_[0] << ", " << rwrist_pos_offset_[1]
              << ", " << rwrist_pos_offset_[2] << "]\n";
}

void TeleopTracker::apply_calibration(const Raw3Point& raw, VR3Point& out) const {
    // Neck: calibrated = inv(initial) * current
    auto neck_q = quat_mul(neck_quat_inv_, raw.quat[2]);

    // Wrists: position rotated by neck inverse minus captured offset;
    // orientation = rot_offset * (neck_inv * current)
    auto lw_pos = quat_rotate(neck_quat_inv_, raw.pos[0]);
    auto rw_pos = quat_rotate(neck_quat_inv_, raw.pos[1]);
    auto lw_q = quat_mul(lwrist_rot_offset_, quat_mul(neck_quat_inv_, raw.quat[0]));
    auto rw_q = quat_mul(rwrist_rot_offset_, quat_mul(neck_quat_inv_, raw.quat[1]));

    // Neck position is synthesized, not tracked: root -> torso_link (+Z),
    // then along the calibrated neck's local Z.
    auto neck_z = quat_rotate(neck_q, {0.0, 0.0, 1.0});

    for (int i = 0; i < 3; ++i) {
        out.position[i]     = lw_pos[i] - lwrist_pos_offset_[i];
        out.position[3 + i] = rw_pos[i] - rwrist_pos_offset_[i];
        out.position[6 + i] = (i == 2 ? kTorsoLinkOffsetZ : 0.0) + kNeckLinkLength * neck_z[i];
    }
    for (int i = 0; i < 4; ++i) {
        out.orientation[i]     = lw_q[i];
        out.orientation[4 + i] = rw_q[i];
        out.orientation[8 + i] = neck_q[i];
    }
}

} // namespace kist
