#include "motion/nav_cmd_receiver.hpp"
#include "motion/input_handler.hpp"

#include <cmath>
#include <iostream>

namespace kist {

NavCmdReceiver& NavCmdReceiver::instance() {
    static NavCmdReceiver inst;
    return inst;
}

bool NavCmdReceiver::start() {
    try {
        sub_.reset(new TwistSub(kNavCmdTopic));
        sub_->InitChannel([this](const void* msg) { on_cmd(msg); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[NavCmdReceiver] DDS subscribe failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[NavCmdReceiver] listening on " << kNavCmdTopic << "\n";
    return true;
}

void NavCmdReceiver::stop() {
    sub_.reset();
}

void NavCmdReceiver::on_cmd(const void* message) {
    const auto& msg = *static_cast<const geometry_msgs::msg::dds_::Twist_*>(message);

    NavCommand cmd;
    cmd.vx   = msg.linear().x();
    cmd.vy   = msg.linear().y();
    cmd.vyaw = msg.angular().z();

    // A NaN/Inf velocity would flow straight into locomotion (std::clamp passes
    // NaN through), so drop the whole command before it reaches nav_buf.
    if (!std::isfinite(cmd.vx) || !std::isfinite(cmd.vy) || !std::isfinite(cmd.vyaw)) {
        auto n = rejected_.fetch_add(1, std::memory_order_relaxed) + 1;
        // 20Hz of garbage must not flood the log: first hit, then 1/100 (~5s).
        if (n == 1 || n % 100 == 0)
            std::cerr << "[NavCmdReceiver] non-finite command rejected (" << n << " total)\n";
        return;
    }

    InputHandler::instance().nav_buf.SetData(cmd);
    // One-shot: announce the first command so it's clear the nav link is up.
    if (received_.fetch_add(1, std::memory_order_relaxed) == 0)
        std::cout << "[NavCmdReceiver] first nav command received — link up\n";
}

} // namespace kist
