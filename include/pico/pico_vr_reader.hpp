#pragma once

#include "common/data_buffer.hpp"
#include "pico/pico_vr_body_pose.hpp"
#include "pico/pico_vr_controller.hpp"
#include <atomic>
#include <thread>

namespace kist {

class PicoVRReader {
public:
    // ── lifecycle ──────────────────────────────────────────────
    static PicoVRReader& instance();
    bool                 start();   // false: daemon absent or SDK init failed
    void                 stop();

    // ── data buffers (read from any thread) ────────────────────
    // Link liveness: the watchdog clears the CONTROLLER buffer when its
    // stream goes stale (consumers safe-stop off "no data"); the BODY
    // buffer holds its last sample so the teleoperated upper body freezes
    // in place through a dropout instead of snapping away and back.
    DataBuffer<PicoVRBodyPose>        body_buf;
    DataBuffer<PicoVRController>      ctrl_buf;

    // ── internal: called from SDK callback ─────────────────────
    void on_body_update(const PicoVRBodyPose& pose);
    void on_controller_update(const PicoVRController& ctrl);

private:
    PicoVRReader() = default;

    void watchdog_loop();

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};
};

} // namespace kist
