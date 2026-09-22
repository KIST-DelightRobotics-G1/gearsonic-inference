#pragma once

#include "control/motor_command.hpp"
#include "control/motion_token.hpp"
#include "planner/motion_sequence_50hz.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace kist {

// Per-tick numeric trace of the whole control path — what the operator
// did, what every stage produced, and what the robot actually did — so
// an incident can be reconstructed afterwards instead of guessed at.
//
// One fixed-width float32 row per 50Hz control tick, written to
// logs/latest.trace (next to the console mirror; overwritten each run).
// The file opens with a text header naming every column, so a reader
// needs nothing but the file (tools/read_trace.py).
//
// Cost model, so the control loop never waits on it:
//  - the control thread fills a row and copies it into a preallocated
//    ring (about 2 KB, a few microseconds) — no allocation, no I/O;
//  - a low-priority thread drains the ring to disk every 200 ms;
//  - if the writer falls behind, new rows are dropped and counted
//    (`trace_dropped` column) — the control thread is never blocked.
class StateTrace {
public:
    // Column layout: kept as a single ordered list so the header and the
    // row packing cannot drift apart.
    struct Row;
    static const std::vector<std::string>& columns();
    static size_t num_columns();

    // What only the controller knows this tick. Everything else (robot
    // state, hands, controller, health flags, teleop buffers, token) is
    // pulled from the module buffers inside record().
    struct TickInfo {
        std::chrono::steady_clock::time_point t0;
        int      wbc_state{0};        // WholeBodyController::State
        int      encoder_mode{-1};    // -1 before the first CONTROL tick
        int      planner_cursor{0};
        const MotionSequence50Hz::Frame* planner_frame{nullptr};
        long     timing_enc_us{0}, timing_dec_us{0}, timing_total_us{0};
    };

    static StateTrace& instance();

    bool start(const std::string& path);
    void stop();
    bool enabled() const { return enabled_.load(); }

    // Control thread, once per tick. No-op when not started.
    void record(const TickInfo& info);

private:
    StateTrace() = default;
    void writer_loop();
    void fill(const TickInfo& info, float* row);

    static constexpr int kRing = 512;           // ~10 s at 50Hz, ~1 MB
    std::vector<float>   ring_;                 // kRing * num_columns
    std::atomic<uint32_t> head_{0};             // rows produced
    std::atomic<uint32_t> tail_{0};             // rows written
    std::atomic<uint32_t> dropped_{0};
    uint64_t              tick_{0};
    std::chrono::steady_clock::time_point start_time_{};

    std::FILE*        file_{nullptr};
    std::thread       writer_;
    std::atomic<bool> enabled_{false};
    std::atomic<bool> stop_{false};
};

} // namespace kist
