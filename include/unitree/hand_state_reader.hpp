#pragma once

#include "common/data_buffer.hpp"
#include "unitree/hand_state.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/HandState_.hpp>

namespace kist {

// Dex3-1 feedback intake: rt/dex3/left/state and rt/dex3/right/state on
// the DDS factory UnitreeStateReader::start() already initialized.
//
// No thread of its own. The SDK subscriber invokes the callback on the
// DDS receive thread, which only converts and stores into the buffer;
// consumers read from wherever they run. There is no watchdog either —
// the single consumer (HandCommandWriter, 100Hz) checks the buffer age
// itself, so a stale/absent hand stream degrades to open-loop commands
// instead of being cleared here.
class HandStateReader {
public:
    static HandStateReader& instance();
    bool start();
    void stop();

    // ── data buffers (read from any thread) ────────────────────
    DataBuffer<HandState> left_buf;
    DataBuffer<HandState> right_buf;

private:
    HandStateReader() = default;

    void on_left_update(const void* message);
    void on_right_update(const void* message);

    using SdkHandState = unitree_hg::msg::dds_::HandState_;
    using HandStateSub = unitree::robot::ChannelSubscriber<SdkHandState>;

    unitree::robot::ChannelSubscriberPtr<SdkHandState> left_sub_;
    unitree::robot::ChannelSubscriberPtr<SdkHandState> right_sub_;
};

} // namespace kist
