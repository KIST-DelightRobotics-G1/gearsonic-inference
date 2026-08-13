// VR link quality monitor — characterizes the PICO stream dropouts without
// the robot. Polls both buffers at 2ms, measures inter-sample gaps per
// stream, and prints one stats line per second plus an immediate '!!'
// marker for every gap beyond 145ms (kept as a fixed measurement bucket
// for comparability across logs; the control-side stale threshold in
// pico_vr_reader.cpp is a separate, larger value).
//
// What the signature tells you:
//   - body AND ctrl gap together      -> link-level (WiFi/AP) problem
//   - body-only gaps, ctrl steady     -> daemon/payload side (body JSON is
//                                        the heavy stream)
//   - gaps periodic (e.g. every ~1s)  -> interference / power-save cadence
//   - gaps grow with movement/distance-> coverage/signal problem
//
// NOTE: the headset stops streaming body when not worn (proximity sensor) —
// wear it or cover the sensor for every run.
//
// Run: ./build/vr_link_monitor   (daemon must be up; Ctrl+C to exit)

#include "pico/pico_vr_reader.hpp"

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

using clock_t_ = std::chrono::steady_clock;

namespace {

struct StreamStats {
    const char* name;
    clock_t_::time_point last_ts{};
    bool   seen{false};
    // per-second window
    int    samples{0};
    double max_gap_ms{0.0};
    int    g20{0}, g50{0}, g145{0};  // gaps in (20,50], (50,145], >145 ms

    void on_sample(clock_t_::time_point ts, double elapsed_s) {
        if (seen && ts != last_ts) {
            double gap = std::chrono::duration<double, std::milli>(ts - last_ts).count();
            ++samples;
            if (gap > max_gap_ms) max_gap_ms = gap;
            if (gap > 145.0) {
                ++g145;
                std::printf("!! %s gap %.0fms  (t=%.1fs)\n", name, gap, elapsed_s);
            } else if (gap > 50.0) {
                ++g50;
            } else if (gap > 20.0) {
                ++g20;
            }
        }
        if (ts != last_ts) {
            last_ts = ts;
            seen    = true;
        }
    }

    void flush(double window_s) {
        std::printf("%s %5.1f Hz | max gap %6.1fms | >20ms:%d >50ms:%d >145ms:%d",
                    name, samples / window_s, max_gap_ms, g20, g50, g145);
        samples = 0;
        max_gap_ms = 0.0;
        g20 = g50 = g145 = 0;
    }
};

}  // namespace

int main() {
    auto& reader = kist::PicoVRReader::instance();
    if (!reader.start())
        return 1;

    std::printf("monitoring (1 line/s; '!!' = gap > 145ms). Ctrl+C to exit.\n");

    StreamStats body{"body"}, ctrl{"ctrl"};
    const auto t0 = clock_t_::now();
    auto window_start = t0;

    while (true) {
        auto now = clock_t_::now();
        double elapsed_s = std::chrono::duration<double>(now - t0).count();

        auto b = reader.body_buf.GetDataWithTime();
        if (b.HasData()) body.on_sample(b.timestamp, elapsed_s);
        auto c = reader.ctrl_buf.GetDataWithTime();
        if (c.HasData()) ctrl.on_sample(c.timestamp, elapsed_s);

        double window_s = std::chrono::duration<double>(now - window_start).count();
        if (window_s >= 1.0) {
            body.flush(window_s);
            std::printf("  ||  ");
            ctrl.flush(window_s);
            std::printf("\n");
            window_start = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
