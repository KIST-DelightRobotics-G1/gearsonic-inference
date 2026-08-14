#pragma once

#include "common/data_buffer.hpp"
#include "control/motion_token.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>

#include <atomic>
#include <cstdint>
#include <thread>

// idl-generated (build/idl_gen, from idl/kist_msgs.idl)
#include "kist_msgs.hpp"

namespace kist {

// 50Hz publisher of the decoder-input token record (rt/kist/motion_token,
// gearsonic -> data collector). Pure observer: it consumes the copy
// WholeBodyController drops into motion_token_buf — the control path never
// touches DDS here, so a stalled write can only delay this thread. Polls at
// 4ms and publishes only when seq advances: exactly one message per decoded
// tick, no duplicates (see motion_token.hpp for the sample semantics).
//
// Requires the unitree ChannelFactory to be initialized first
// (UnitreeStateReader::start does that).
class MotionTokenPublisher {
public:
    static MotionTokenPublisher& instance();

    // `source` is WholeBodyController::motion_token_buf (wired by the
    // facade, mirroring UnitreeCommandWriter's source pattern).
    bool start(const DataBuffer<MotionTokenSample>* source);
    void stop();

private:
    MotionTokenPublisher() = default;

    void loop();

    using SdkMsg = kist_msgs::MotionTokenState;
    unitree::robot::ChannelPublisherPtr<SdkMsg> pub_;

    const DataBuffer<MotionTokenSample>* source_{nullptr};
    uint64_t last_seq_{0};  // 0 = nothing published yet (seq starts at 1)

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
