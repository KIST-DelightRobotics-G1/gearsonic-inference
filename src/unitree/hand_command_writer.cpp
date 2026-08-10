#include "unitree/hand_command_writer.hpp"

#include "common/thread_priority.hpp"
#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"
#include "teleop/teleop_tracker.hpp"

#include "control/robot_ownership.hpp"
#include "vla/vla_token_receiver.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>

namespace kist {

static const std::string kLeftCmdTopic  = "rt/dex3/left/cmd";
static const std::string kRightCmdTopic = "rt/dex3/right/cmd";
static constexpr auto    kPublishPeriod = std::chrono::milliseconds(10);  // 100 Hz
static constexpr int     kMotorCount    = 7;
// PD gains follow the SDK example's gripHand pose — enough authority to
// hold a grasp against light loads without heating the tiny 4010 actuators.
static constexpr float   kHandKp        = 1.5f;
static constexpr float   kHandKd        = 0.1f;
// Fist keeps holding this long after calibration engages, so the hands
// don't pop open the moment the B-hold gesture lands.
static constexpr auto    kFistReleaseDelay = std::chrono::milliseconds(500);

// RIS_Mode byte layout: [id:4][status:3][timeout:1] (SDK example).
static inline uint8_t ris_mode(uint8_t id, bool enable) {
    uint8_t mode = static_cast<uint8_t>(id & 0x0F);
    mode |= static_cast<uint8_t>((enable ? 0x01 : 0x00) & 0x07) << 4;
    mode |= static_cast<uint8_t>((enable ? 0x00 : 0x01) & 0x01) << 7;
    return mode;
}

// Blend "open" (input=0) to "closed" (input=1) between explicit endpoints,
// with a per-motor group input. Motor 0..2 are the thumb, driven by the
// controller's grip axis; 3..6 are the index+middle fingers, driven by
// the trigger. This gives the operator two independent grasps — a
// thumb-only pinch (grip only), a finger cage (trigger only), or a full
// power grip (both) — mapped onto the natural hand-on-controller pose.
//
// The endpoints come from kDex3{Left,Right}{Open,Close} — thumb crosses
// zero (bidirectional joints), fingers pivot around zero on one side
// (unidirectional flexion). See hand_command.hpp for the empirical
// convention.
static HandCommand from_grip_and_trigger(double grip, double trigger, bool is_left) {
    grip    = std::clamp(grip,    0.0, 1.0);
    trigger = std::clamp(trigger, 0.0, 1.0);
    HandCommand cmd;
    for (int i = 0; i < kMotorCount; ++i) {
        float open_q  = is_left ? kDex3LeftOpen[i]  : kDex3RightOpen[i];
        float close_q = is_left ? kDex3LeftClose[i] : kDex3RightClose[i];
        double t = (i < kFingerBegin) ? grip : trigger;
        cmd.q[i]  = open_q + static_cast<float>(t) * (close_q - open_q);
        cmd.kp[i] = kHandKp;
        cmd.kd[i] = kHandKd;
    }
    cmd.enabled = true;
    return cmd;
}

// URDF joint limits from the SDK dex3 example (the reference table in
// hand_command.hpp) — VLA joint targets are clamped into them.
static constexpr std::array<float, 7> kDex3LeftMin  = {-1.05f, -0.724f,  0.00f, -1.57f, -1.75f, -1.57f, -1.75f};
static constexpr std::array<float, 7> kDex3LeftMax  = { 1.05f,  1.05f,   1.75f,  0.00f,  0.00f,  0.00f,  0.00f};
static constexpr std::array<float, 7> kDex3RightMin = {-1.05f, -1.05f,  -1.75f,  0.00f,  0.00f,  0.00f,  0.00f};
static constexpr std::array<float, 7> kDex3RightMax = { 1.05f,  0.742f,  0.00f,  1.57f,  1.75f,  1.57f,  1.75f};

// External VLA hand targets: already Dex3 motor order (thumb x3, index x2,
// middle x2), absolute joint positions — clamp and publish with the same
// PD gains as trigger tracking.
static HandCommand from_vla_joints(const std::array<float, 7>& q, bool is_left) {
    const auto& lo = is_left ? kDex3LeftMin : kDex3RightMin;
    const auto& hi = is_left ? kDex3LeftMax : kDex3RightMax;
    HandCommand cmd;
    for (int i = 0; i < kMotorCount; ++i) {
        cmd.q[i]  = std::clamp(q[i], lo[i], hi[i]);
        cmd.kp[i] = kHandKp;
        cmd.kd[i] = kHandKd;
    }
    cmd.enabled = true;
    return cmd;
}

static HandCommand stop_command() {
    HandCommand cmd;
    for (int i = 0; i < kMotorCount; ++i) {
        cmd.q[i]  = 0.0f;
        cmd.kp[i] = 0.0f;
        cmd.kd[i] = 0.0f;
    }
    cmd.enabled = false;
    return cmd;
}

HandCommandWriter& HandCommandWriter::instance() {
    static HandCommandWriter inst;
    return inst;
}

bool HandCommandWriter::start() {
    // ChannelFactory must already be initialized by UnitreeStateReader::start().
    left_pub_  = std::make_shared<unitree::robot::ChannelPublisher<SdkHandCmd>>(kLeftCmdTopic);
    right_pub_ = std::make_shared<unitree::robot::ChannelPublisher<SdkHandCmd>>(kRightCmdTopic);
    left_pub_->InitChannel();
    right_pub_->InitChannel();

    stop_        = false;
    loop_thread_ = std::thread(&HandCommandWriter::loop, this);
    try_realtime_priority(loop_thread_, 50, "HandCommandWriter");
    std::cout << "[HandCommandWriter] started (100 Hz, publishing to " << kLeftCmdTopic
              << " and " << kRightCmdTopic << ")\n";
    return true;
}

void HandCommandWriter::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
    send_stop();
    std::cout << "[HandCommandWriter] stopped (stop command sent)\n";
}

void HandCommandWriter::send_stop() {
    if (!left_pub_ || !right_pub_)
        return;
    auto s = stop_command();
    for (int n = 0; n < 5; ++n) {
        publish(s, s);
        std::this_thread::sleep_for(kPublishPeriod);
    }
}

void HandCommandWriter::fill_msg(SdkHandCmd& msg, const HandCommand& cmd) {
    msg.motor_cmd().resize(kMotorCount);
    for (int i = 0; i < kMotorCount; ++i) {
        msg.motor_cmd().at(i).mode() = ris_mode(static_cast<uint8_t>(i), cmd.enabled);
        msg.motor_cmd().at(i).q()    = cmd.q[i];
        msg.motor_cmd().at(i).dq()   = 0.0f;
        msg.motor_cmd().at(i).tau()  = 0.0f;
        msg.motor_cmd().at(i).kp()   = cmd.kp[i];
        msg.motor_cmd().at(i).kd()   = cmd.kd[i];
    }
}

void HandCommandWriter::publish(const HandCommand& left, const HandCommand& right) {
    SdkHandCmd lmsg, rmsg;
    fill_msg(lmsg, left);
    fill_msg(rmsg, right);
    left_pub_->Write(lmsg);
    right_pub_->Write(rmsg);
}

void HandCommandWriter::loop() {
    using clock = std::chrono::steady_clock;

    bool was_calibrated = false;
    clock::time_point calibrated_at{};

    while (!stop_) {
        auto t0 = clock::now();

        bool calibrated = TeleopTracker::instance().calibrated();
        if (calibrated && !was_calibrated)
            calibrated_at = t0;
        was_calibrated = calibrated;
        bool hold_fist = !calibrated || (t0 - calibrated_at) < kFistReleaseDelay;

        HandCommand left, right;
        if (InputHandler::instance().estop()) {
            left = right = stop_command();
        } else if (auto vla = VlaTokenReceiver::instance().latent_buf.GetDataWithTime();
                   RobotOwnership::instance().owner() == RobotOwnership::Owner::kVla &&
                   vla.HasData() && vla.GetAgeMs() <= kVlaTokenHoldMs) {
            // VLA owns the robot (first-come arbitration, see
            // robot_ownership.hpp): its hand targets replace the VR trigger
            // mapping. Under teleop ownership this branch never fires.
            left  = from_vla_joints(vla.data->left_hand,  /*is_left=*/true);
            right = from_vla_joints(vla.data->right_hand, /*is_left=*/false);
        } else if (hold_fist) {
            // Before teleop calibration the trigger/grip axes still serve
            // the locomotion UI (trigger+A/B height, extended mode list),
            // so the hands hold a closed fist instead of tracking them.
            left  = from_grip_and_trigger(1.0, 1.0, /*is_left=*/true);
            right = from_grip_and_trigger(1.0, 1.0, /*is_left=*/false);
        } else if (auto ctrl = PicoVRReader::instance().ctrl_buf.GetData()) {
            left  = from_grip_and_trigger(ctrl->left_grip,  ctrl->left_trigger,  /*is_left=*/true);
            right = from_grip_and_trigger(ctrl->right_grip, ctrl->right_trigger, /*is_left=*/false);
        } else {
            // VR link lost: hold the open pose (all inputs=0), low PD.
            left  = from_grip_and_trigger(0.0, 0.0, /*is_left=*/true);
            right = from_grip_and_trigger(0.0, 0.0, /*is_left=*/false);
        }

        publish(left, right);
        std::this_thread::sleep_until(t0 + kPublishPeriod);
    }
}

} // namespace kist
