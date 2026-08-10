#pragma once

#include "unitree/hand_command.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/HandCmd_.hpp>

#include <atomic>
#include <thread>

namespace kist {

// 100Hz Dex3-1 hand publisher. Two topics — rt/dex3/left/cmd and
// rt/dex3/right/cmd — served from a single loop; the DDS factory is
// shared with UnitreeStateReader/UnitreeCommandWriter (initialized by
// UnitreeStateReader::start()).
//
// Input source: VR trigger values from PicoVRReader::ctrl_buf, mapped
// linearly onto each hand's URDF joint range (open -> closed as the
// trigger presses in). InputHandler is not on the path: the trigger
// stream is orthogonal to the movement command it produces.
//
// Gated by TeleopTracker::calibrated(): until teleop calibration engages,
// the trigger/grip axes belong to the locomotion UI, so both hands hold
// a closed fist; tracking starts kFistReleaseDelay after calibration
// (the fist doesn't pop open the instant the gesture lands).
//
// SAFETY:
//  - InputHandler::estop() latched -> writer publishes stop mode
//    (kp=0, kd=0, timeout bit) every tick.
//  - stop() sends a stop burst before joining.
//  - VR link lost (empty ctrl_buf) -> hands hold open pose with low PD.
class HandCommandWriter {
public:
    static HandCommandWriter& instance();
    bool start();
    void stop();

    // Immediately publish a stop command to both hands (safe shutdown).
    void send_stop();

private:
    HandCommandWriter() = default;

    void loop();
    void publish(const HandCommand& left, const HandCommand& right);
    void fill_msg(unitree_hg::msg::dds_::HandCmd_& msg, const HandCommand& cmd);

    using SdkHandCmd = unitree_hg::msg::dds_::HandCmd_;
    unitree::robot::ChannelPublisherPtr<SdkHandCmd> left_pub_;
    unitree::robot::ChannelPublisherPtr<SdkHandCmd> right_pub_;

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
