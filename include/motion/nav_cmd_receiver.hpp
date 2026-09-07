#pragma once

#include "motion/nav_command.hpp"

#include <unitree/idl/ros2/Twist_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <cstdint>

namespace kist {

// Topic carrying the base-frame velocity command from the navigation planner
// (kist-navigation-planner publishes a ROS Twist here).
inline constexpr const char* kNavCmdTopic = "rt/kist/nav/cmd_vel";

// DDS Rx for the external navigation velocity command (vx, vy, vyaw), the
// missing bridge between kist-navigation-planner and this locomotion stack.
//
// Pure intake, mirroring VlaTokenReceiver: each Twist (linear.x=vx, linear.y=vy,
// angular.z=vyaw) is validated (finite) and written into InputHandler::nav_buf
// (latest-wins). Non-finite samples are dropped so the arbitration never sees
// NaN/Inf. Operational meaning — stick arbitration, staleness fallback, e-stop —
// still lives in the consumer (input_handler.cpp).
//
// Requires the unitree ChannelFactory to be initialized first
// (UnitreeStateReader::start does that).
class NavCmdReceiver {
public:
    static NavCmdReceiver& instance();
    bool start();
    void stop();

    uint64_t received() const { return received_.load(); }  // accepted commands (rate probe)
    uint64_t rejected() const { return rejected_.load(); }  // dropped by validation

private:
    NavCmdReceiver() = default;

    void on_cmd(const void* message);

    using TwistSub = unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::Twist_>;
    unitree::robot::ChannelSubscriberPtr<geometry_msgs::msg::dds_::Twist_> sub_;

    std::atomic<uint64_t> received_{0};
    std::atomic<uint64_t> rejected_{0};
};

} // namespace kist
