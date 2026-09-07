// Standalone Rx probe for the navigation velocity command (vx, vy, vyaw).
//
// Pair it with kist-navigation-planner publishing on rt/kist/nav/cmd_vel to
// verify the wire + sign/frame conventions WITHOUT the robot moving:
//
//   ./nav_receiver_probe [domain_id=0] [network_interface=<SDK default>]
//
// It starts NavCmdReceiver (which writes InputHandler::nav_buf) and prints a 1 Hz
// line with the receive rate and the newest command. Sanity checks:
//   - goal ahead of the robot  -> vx > 0
//   - goal to the robot's left -> vyaw > 0 (CCW)
//   - no goal / arrived        -> zeros
//
// Note: "lo" is not multicast-capable — for same-host tests leave the interface
// at the default (all interfaces) so discovery works.

#include "motion/nav_cmd_receiver.hpp"
#include "motion/input_handler.hpp"

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
    const int         domain = (argc >= 2) ? std::atoi(argv[1]) : 0;
    const std::string iface  = (argc >= 3) ? argv[2] : "";

    std::setvbuf(stdout, nullptr, _IOLBF, 0);  // line-buffered under redirects
    std::signal(SIGINT, [](int) { g_quit = true; });

    unitree::robot::ChannelFactory::Instance()->Init(domain, iface);

    auto& rx = kist::NavCmdReceiver::instance();
    if (!rx.start())
        return 1;
    std::printf("probe: domain=%d iface=%s — waiting for nav commands (Ctrl+C to quit)\n",
                domain, iface.c_str());

    uint64_t last_count = 0;
    while (!g_quit) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const uint64_t count = rx.received();
        auto v = kist::InputHandler::instance().nav_buf.GetDataWithTime();
        if (v.HasData()) {
            const auto& c = *v.data;
            std::printf("rx %llu msgs (+%llu/s) | age %.1fms | vx % .3f  vy % .3f  vyaw % .3f  | rejected %llu\n",
                        (unsigned long long)count, (unsigned long long)(count - last_count),
                        v.GetAgeMs(), c.vx, c.vy, c.vyaw,
                        (unsigned long long)rx.rejected());
        } else {
            std::printf("rx %llu msgs — no data yet\n", (unsigned long long)count);
        }
        last_count = count;
    }

    rx.stop();
    return 0;
}
