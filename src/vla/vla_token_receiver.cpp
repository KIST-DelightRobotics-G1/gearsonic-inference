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
    } catch (const std::exception& e) {
        std::cerr << "[VlaTokenReceiver] DDS subscribe failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[VlaTokenReceiver] listening on " << kVlaLatentActionTopic << "\n";
    return true;
}

void VlaTokenReceiver::stop() {
    latent_sub_.reset();
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

} // namespace kist
