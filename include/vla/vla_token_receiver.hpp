#pragma once

#include "common/data_buffer.hpp"
#include "common/event_queue.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

// idl-generated (build/idl_gen, from idl/kist_latent_action.idl)
#include "kist_latent_action.hpp"

namespace kist {

// DDS topics shared with kist-vla-inference — must match the constants in
// its kist_vla/io/dds.py. Named in the rt/kist/* convention used by
// kist-ext-sensor-io.
inline constexpr const char* kVlaLatentActionTopic = "rt/kist/latent_action";
inline constexpr const char* kVlaWbcCommandTopic   = "rt/kist/wbc_command";

// Freshness thresholds for the external token stream (see
// WholeBodyController::tick_vla_control):
//   fresh  — a token this recent switches the controller into VLA mode
//   hold   — beyond this age in VLA mode, the stream counts as lost
inline constexpr double kVlaTokenFreshMs = 200.0;
inline constexpr double kVlaTokenHoldMs  = 500.0;

// Plain-data snapshot of the newest kist_msgs::LatentActionStep.
// Hand joints are Dex3 motor order (thumb x3, index x2, middle x2) —
// the same layout HandCommand::q uses.
struct VlaLatentAction {
    std::array<float, 64> token{};
    std::array<float, 7>  left_hand{};
    std::array<float, 7>  right_hand{};
    int64_t  frame_index{0};
    int64_t  stamp_ns{0};
    uint64_t seq{0};
};

// Snapshot of the newest kist_msgs::WbcCommand.
struct VlaCommand {
    bool     start{false};
    bool     stop{false};
    bool     planner{false};
    uint64_t seq{0};
};

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
