#pragma once

#include "common/data_buffer.hpp"
#include "vla/vla_latent_action.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <cstdint>

// idl-generated (build/idl_gen, from idl/kist_latent_action.idl)
#include "kist_latent_action.hpp"

namespace kist {

// DDS Rx for the VLA latent-action stream (kist-vla-inference -> here).
//
// Data intake mirroring UnitreeStateReader's shape: the token stream is
// state ("newest wins") and lands in a latest-wins DataBuffer. The receiver
// validates each sample itself — any non-finite float (token or hand
// joints) drops the whole sample before it reaches the buffer, so
// consumers never see NaN/Inf. Operational meaning (mode switching,
// staleness policy, safety) still lives in the consumers —
// WholeBodyController and HandCommandWriter.
//
// Requires the unitree ChannelFactory to be initialized first
// (UnitreeStateReader::start does that).
class VlaTokenReceiver {
public:
    static VlaTokenReceiver& instance();
    bool start();
    void stop();

    DataBuffer<VlaLatentAction> latent_buf;

    // Messages received so far (monotonic; produce-side truth for rate probes)
    uint64_t latent_received() const { return latent_received_.load(); }

    // Samples dropped by validation (non-finite values)
    uint64_t latent_rejected() const { return latent_rejected_.load(); }

private:
    VlaTokenReceiver() = default;

    void on_latent_action(const void* message);

    using LatentSub = unitree::robot::ChannelSubscriber<kist_msgs::LatentActionStep>;

    unitree::robot::ChannelSubscriberPtr<kist_msgs::LatentActionStep> latent_sub_;

    std::atomic<uint64_t> latent_received_{0};
    std::atomic<uint64_t> latent_rejected_{0};
};

} // namespace kist
