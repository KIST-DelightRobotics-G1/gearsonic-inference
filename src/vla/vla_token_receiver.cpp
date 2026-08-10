#include "vla/vla_token_receiver.hpp"

#include <algorithm>
#include <iostream>

namespace kist {

VlaTokenReceiver& VlaTokenReceiver::instance() {
    static VlaTokenReceiver inst;
    return inst;
}

bool VlaTokenReceiver::start() {
    try {
        latent_sub_.reset(new LatentSub(kVlaLatentActionTopic));
        latent_sub_->InitChannel(
            [this](const void* msg) { on_latent_action(msg); }, 1);

        // Commands are events — give the SDK dispatch a little depth too,
        // so a burst isn't conflated away below our queue.
        command_sub_.reset(new CommandSub(kVlaWbcCommandTopic));
        command_sub_->InitChannel(
            [this](const void* msg) { on_command(msg); }, 8);
    } catch (const std::exception& e) {
        std::cerr << "[VlaTokenReceiver] DDS subscribe failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[VlaTokenReceiver] listening on " << kVlaLatentActionTopic
              << " and " << kVlaWbcCommandTopic << "\n";
    return true;
}

void VlaTokenReceiver::stop() {
    latent_sub_.reset();
    command_sub_.reset();
}

void VlaTokenReceiver::on_latent_action(const void* message) {
    const auto& msg = *static_cast<const kist_msgs::LatentActionStep*>(message);

    VlaLatentAction out;
    out.token       = msg.token_state();
    out.left_hand   = msg.left_hand_joints();
    out.right_hand  = msg.right_hand_joints();
    out.frame_index = msg.frame_index();
    out.stamp_ns    = msg.stamp_ns();
    out.seq         = msg.seq();

    latent_buf.SetData(std::move(out));
    latent_received_.fetch_add(1, std::memory_order_relaxed);
}

void VlaTokenReceiver::on_command(const void* message) {
    const auto& msg = *static_cast<const kist_msgs::WbcCommand*>(message);

    VlaCommand out;
    out.start   = msg.start();
    out.stop    = msg.stop();
    out.planner = msg.planner();
    out.seq     = msg.seq();

    std::cout << "[VlaTokenReceiver] command: start=" << out.start
              << " stop=" << out.stop << " planner=" << out.planner
              << " (seq " << out.seq << ")\n";

    if (auto evicted = command_queue_.push(std::move(out))) {
        std::cerr << "[VlaTokenReceiver] command queue full — evicted oldest "
                     "(seq " << evicted->seq << ")\n";
    }
}

} // namespace kist
