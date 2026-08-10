// Standalone Rx probe for the VLA latent-action DDS stream.
//
// Pair it with kist-vla-inference publishing in dds mode to verify
// cross-language interop without the robot:
//
//   ./vla_receiver_probe [domain_id=0] [network_interface=<SDK default>]
//
// Note: "lo" is not multicast-capable — for same-host tests leave the
// interface at the default (all interfaces) so discovery works.
//
// Prints a 1 Hz line with the receive rate and the newest token/hand
// values; commands are logged by the receiver as they arrive.

#include "vla/vla_token_receiver.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

static std::atomic<bool> g_quit{false};

int main(int argc, char** argv) {
    const int domain = (argc >= 2) ? std::atoi(argv[1]) : 0;
    const std::string iface = (argc >= 3) ? argv[2] : "";

    std::setvbuf(stdout, nullptr, _IOLBF, 0);  // line-buffered under redirects

    std::signal(SIGINT, [](int) { g_quit = true; });

    unitree::robot::ChannelFactory::Instance()->Init(domain, iface);

    auto& rx = kist::VlaTokenReceiver::instance();
    if (!rx.start())
        return 1;
    std::printf("probe: domain=%d iface=%s — waiting for tokens (Ctrl+C to quit)\n",
                domain, iface.c_str());

    uint64_t last_count = 0;
    while (!g_quit) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        for (const auto& cmd : rx.take_commands())
            std::printf("command drained: seq %llu start=%d stop=%d planner=%d\n",
                        (unsigned long long)cmd.seq, cmd.start, cmd.stop, cmd.planner);

        const uint64_t count = rx.latent_received();
        auto v = rx.latent_buf.GetDataWithTime();
        if (v.HasData()) {
            const auto& a = *v.data;
            std::printf(
                "rx %llu msgs (+%llu/s) | seq %llu frame %lld age %.1fms | "
                "token[0:4] % .3f % .3f % .3f % .3f | L0 % .2f R0 % .2f\n",
                (unsigned long long)count, (unsigned long long)(count - last_count),
                (unsigned long long)a.seq, (long long)a.frame_index, v.GetAgeMs(),
                a.token[0], a.token[1], a.token[2], a.token[3],
                a.left_hand[0], a.right_hand[0]);
        } else {
            std::printf("rx %llu msgs — no data yet\n", (unsigned long long)count);
        }
        last_count = count;
    }

    rx.stop();
    return 0;
}
