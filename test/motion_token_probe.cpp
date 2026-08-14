// Standalone Rx probe for the motion-token record stream
// (rt/kist/motion_token, published by the running gearsonic process).
//
// Stands in for the data collector to verify the recording side without
// it — checks rate, seq continuity (gaps = non-CONTROL ticks by design;
// mid-CONTROL losses show up as missing seq at a steady 50 Hz), and the
// mode tags used for training-data filtering:
//
//   ./motion_token_probe [domain_id=0] [network_interface=<SDK default>]

#include "control/motion_token.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// idl-generated (build/idl_gen, from idl/kist_msgs.idl)
#include "kist_msgs.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>

static std::atomic<bool> g_quit{false};

namespace {

struct Stats {
    std::mutex mtx;
    uint64_t count{0};
    uint64_t last_seq{0};
    uint64_t seq_gaps{0};  // seq jumps > 1 (expected across non-CONTROL periods)
    kist_msgs::MotionTokenState newest;
};
Stats g_stats;

void on_msg(const void* message) {
    const auto& msg = *static_cast<const kist_msgs::MotionTokenState*>(message);
    std::lock_guard<std::mutex> l(g_stats.mtx);
    ++g_stats.count;
    if (g_stats.last_seq != 0 && msg.seq() > g_stats.last_seq + 1)
        ++g_stats.seq_gaps;
    g_stats.last_seq = msg.seq();
    g_stats.newest   = msg;
}

const char* mode_name(uint8_t m) {
    switch (m) {
        case 0: return "normal";
        case 1: return "teleop";
        case 2: return "vla";
        case 3: return "recovering";
    }
    return "?";
}

}  // namespace

int main(int argc, char** argv) {
    const int domain = (argc >= 2) ? std::atoi(argv[1]) : 0;
    const std::string iface = (argc >= 3) ? argv[2] : "";

    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::signal(SIGINT, [](int) { g_quit = true; });

    unitree::robot::ChannelFactory::Instance()->Init(domain, iface);

    unitree::robot::ChannelSubscriber<kist_msgs::MotionTokenState> sub(
        kist::kMotionTokenTopic);
    sub.InitChannel(on_msg, 1);
    std::printf("probe: domain=%d iface=%s — waiting for tokens (Ctrl+C to quit)\n",
                domain, iface.c_str());

    uint64_t last_count = 0;
    while (!g_quit) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::lock_guard<std::mutex> l(g_stats.mtx);
        if (g_stats.count == 0) {
            std::printf("rx 0 msgs — no data yet\n");
            continue;
        }
        const auto& m = g_stats.newest;
        std::printf(
            "rx %llu msgs (+%llu/s) | seq %llu gaps %llu | arbiter %s enc %u | "
            "token[0:4] % .3f % .3f % .3f % .3f\n",
            (unsigned long long)g_stats.count,
            (unsigned long long)(g_stats.count - last_count),
            (unsigned long long)m.seq(), (unsigned long long)g_stats.seq_gaps,
            mode_name(m.arbiter_mode()), (unsigned)m.encoder_mode(),
            m.token_state()[0], m.token_state()[1],
            m.token_state()[2], m.token_state()[3]);
        last_count = g_stats.count;
    }
    return 0;
}
