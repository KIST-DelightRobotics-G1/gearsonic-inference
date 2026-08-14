#include "collector/motion_token_publisher.hpp"

#include <chrono>
#include <iostream>

namespace kist {

static constexpr auto kPollPeriod = std::chrono::milliseconds(4);

MotionTokenPublisher& MotionTokenPublisher::instance() {
    static MotionTokenPublisher inst;
    return inst;
}

bool MotionTokenPublisher::start(const DataBuffer<MotionTokenSample>* source) {
    source_ = source;
    try {
        pub_.reset(new unitree::robot::ChannelPublisher<SdkMsg>(kMotionTokenTopic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[MotionTokenPublisher] DDS publish init failed: " << e.what() << "\n";
        return false;
    }

    stop_        = false;
    loop_thread_ = std::thread(&MotionTokenPublisher::loop, this);
    std::cout << "[MotionTokenPublisher] started (on-change, publishing to "
              << kMotionTokenTopic << ")\n";
    return true;
}

void MotionTokenPublisher::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
    pub_.reset();
}

void MotionTokenPublisher::loop() {
    while (!stop_) {
        auto s = source_->GetData();
        if (s && s->seq != last_seq_) {
            last_seq_ = s->seq;

            SdkMsg msg;
            msg.seq()      = s->seq;
            msg.stamp_ns() = s->stamp_ns;
            for (size_t i = 0; i < s->token.size(); ++i)
                msg.token_state()[i] = s->token[i];
            msg.arbiter_mode() = s->arbiter_mode;
            msg.encoder_mode() = s->encoder_mode;
            pub_->Write(msg);
        }
        std::this_thread::sleep_for(kPollPeriod);
    }
}

} // namespace kist
