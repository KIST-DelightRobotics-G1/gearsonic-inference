#include "vla/vla_token_receiver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace kist {

// A single NaN/Inf slipping through poisons the decoder input (token) or
// reaches the hand motors (std::clamp passes NaN through), so every float
// in the sample must be finite before it is published to consumers.
template <size_t N>
static bool all_finite(const std::array<float, N>& v) {
    return std::all_of(v.begin(), v.end(),
                       [](float x) { return std::isfinite(x); });
}

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

    if (!all_finite(out.token) || !all_finite(out.left_hand) ||
        !all_finite(out.right_hand)) {
        auto n = latent_rejected_.fetch_add(1, std::memory_order_relaxed) + 1;
        // 50Hz of garbage must not flood the log: first hit, then 1/250 (~5s).
        if (n == 1 || n % 250 == 0)
            std::cerr << "[VlaTokenReceiver] non-finite sample rejected"
                      << " (seq " << out.seq << ", " << n << " total)\n";
        return;
    }

    latent_buf.SetData(std::move(out));
    latent_received_.fetch_add(1, std::memory_order_relaxed);
}

} // namespace kist
