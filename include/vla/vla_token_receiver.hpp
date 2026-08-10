#pragma once

#include "common/data_buffer.hpp"
#include "common/event_queue.hpp"
#include "vla/vla_action.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <cstdint>
#include <vector>

// idl-generated (build/idl_gen, from idl/kist_latent_action.idl)
#include "kist_latent_action.hpp"

namespace kist {

// DDS Rx for the VLA latent-action stream (kist-vla-inference -> here).
//
// Pure data intake, mirroring UnitreeStateReader's shape: the token stream
// is state ("newest wins") and lands in a latest-wins DataBuffer; commands
// are events (each one matters) and land in a small bounded queue that
// consumers drain. Operational meaning (mode switching, staleness policy,
// safety) lives in the consumers — WholeBodyController and
// HandCommandWriter.
//
// Requires the unitree ChannelFactory to be initialized first
// (UnitreeStateReader::start does that).
class VlaTokenReceiver {
public:
    static VlaTokenReceiver& instance();
    bool start();
    void stop();

    DataBuffer<VlaLatentAction> latent_buf;

    // Drain all commands received since the last call, in arrival order.
    // Bounded EventQueue: if a burst overflows it, the OLDEST command is
    // evicted with a warning — never silently.
    std::vector<VlaCommand> take_commands() { return command_queue_.drain(); }

    // Messages received so far (monotonic; produce-side truth for rate probes)
    uint64_t latent_received() const { return latent_received_.load(); }

private:
    VlaTokenReceiver() = default;

    void on_latent_action(const void* message);
    void on_command(const void* message);

    using LatentSub  = unitree::robot::ChannelSubscriber<kist_msgs::LatentActionStep>;
    using CommandSub = unitree::robot::ChannelSubscriber<kist_msgs::WbcCommand>;

    unitree::robot::ChannelSubscriberPtr<kist_msgs::LatentActionStep> latent_sub_;
    unitree::robot::ChannelSubscriberPtr<kist_msgs::WbcCommand>       command_sub_;

    EventQueue<VlaCommand> command_queue_{16};

    std::atomic<uint64_t> latent_received_{0};
};

} // namespace kist
