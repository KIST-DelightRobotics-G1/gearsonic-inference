#pragma once

#include "collector/record_control.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>

// idl-generated (build/idl_gen, from idl/kist_msgs.idl)
#include "kist_msgs.hpp"

namespace kist {

// VR-button recording control: turns InputHandler's A/B taps into
// RecordCommand messages for kist-data-collector. One-way — the collector
// shows the real recording state on its own console; this side keeps a
// local mirror of what it sent so the gestures stay consistent.
//
//   B tap    -> start      B double -> stop
//   A tap    -> success    A double -> fail    A triple -> partial   (after a stop)
//
// No thread of its own: taps arrive on the InputHandler thread (a DDS
// write is microseconds). Requires the ChannelFactory
// (UnitreeStateReader::start) and InputHandler::start first.
class RecordCommander {
public:
    static constexpr const char* kCmdTopic = "rt/kist/record_cmd";

    static RecordCommander& instance();
    bool start();
    void stop();

    // Local mirror of the collector state (what the last sent command
    // implies) — the decision table input and the trace column.
    RecordState state() const { return assumed_.load(); }

    // Entry point from InputHandler's tap sink.
    void on_tap(TapButton button, TapKind kind);

private:
    RecordCommander() = default;
    void publish(RecordAction action);

    using CmdPub = unitree::robot::ChannelPublisher<kist_msgs::RecordCommand>;
    unitree::robot::ChannelPublisherPtr<kist_msgs::RecordCommand> pub_;

    std::mutex               pub_mutex_;
    uint64_t                 seq_{0};
    std::atomic<RecordState> assumed_{RecordState::Idle};
    std::atomic<bool>        started_{false};
};

} // namespace kist
