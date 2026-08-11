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
// Pure data intake, mirroring UnitreeStateReader's shape: the token stream
// is state ("newest wins") and lands in a latest-wins DataBuffer.
// Operational meaning (mode switching, staleness policy, safety) lives in
// the consumers — WholeBodyController and HandCommandWriter.
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

private:
    VlaTokenReceiver() = default;

    void on_latent_action(const void* message);

    using LatentSub = unitree::robot::ChannelSubscriber<kist_msgs::LatentActionStep>;

    unitree::robot::ChannelSubscriberPtr<kist_msgs::LatentActionStep> latent_sub_;

    std::atomic<uint64_t> latent_received_{0};
};

} // namespace kist
