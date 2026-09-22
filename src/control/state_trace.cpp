#include "control/state_trace.hpp"
#include "control/state_trace_columns.hpp"

#include "control/control_arbiter.hpp"
#include "control/whole_body_controller.hpp"
#include "pico/pico_vr_reader.hpp"
#include "teleop/teleop_tracker.hpp"
#include "unitree/hand_command_writer.hpp"
#include "unitree/hand_state_reader.hpp"
#include "unitree/motor_health_monitor.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iostream>

namespace kist {

const std::vector<std::string>& StateTrace::columns() {
    static const std::vector<std::string> cols = state_trace_columns();
    return cols;
}
size_t StateTrace::num_columns() { return columns().size(); }

// ─── lifecycle ────────────────────────────────────────────────────────────────

StateTrace& StateTrace::instance() {
    static StateTrace inst;
    return inst;
}

bool StateTrace::start(const std::string& path) {
    if (enabled_) return true;
    std::error_code ec;
    auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent, ec);

    file_ = std::fopen(path.c_str(), "wb");
    if (!file_) {
        std::cerr << "[StateTrace] cannot open " << path << " — trace disabled\n";
        return false;
    }
    // Text header, then raw little-endian float32 rows.
    std::fprintf(file_, "STATETRACE v1\ncolumns=%zu\nrate_hz=50\n", num_columns());
    const auto& cols = columns();
    for (size_t i = 0; i < cols.size(); ++i)
        std::fprintf(file_, "%s%s", i ? "," : "", cols[i].c_str());
    std::fprintf(file_, "\nEND\n");
    std::fflush(file_);

    ring_.assign(static_cast<size_t>(kRing) * num_columns(), 0.0f);
    head_ = tail_ = dropped_ = 0;
    tick_ = 0;
    start_time_ = std::chrono::steady_clock::now();
    stop_    = false;
    enabled_ = true;
    writer_  = std::thread(&StateTrace::writer_loop, this);
    std::cout << "[StateTrace] recording " << num_columns() << " columns/tick to " << path << "\n";
    return true;
}

void StateTrace::stop() {
    if (!enabled_) return;
    enabled_ = false;
    stop_    = true;
    if (writer_.joinable()) writer_.join();
    if (file_) { std::fclose(file_); file_ = nullptr; }
    std::cout << "[StateTrace] stopped (" << tick_ << " ticks, " << dropped_.load() << " dropped)\n";
}

void StateTrace::writer_loop() {
    const size_t ncol = num_columns();
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        uint32_t head = head_.load(std::memory_order_acquire);
        uint32_t tail = tail_.load(std::memory_order_relaxed);
        while (tail != head) {
            // contiguous run up to the ring end
            uint32_t idx = tail % kRing;
            uint32_t n   = std::min<uint32_t>(head - tail, kRing - idx);
            std::fwrite(ring_.data() + static_cast<size_t>(idx) * ncol, sizeof(float), static_cast<size_t>(n) * ncol, file_);
            tail += n;
        }
        tail_.store(tail, std::memory_order_release);
        std::fflush(file_);
        if (stop_ && head_.load(std::memory_order_acquire) == tail) break;
    }
}

// ─── the row ──────────────────────────────────────────────────────────────────

void StateTrace::record(const TickInfo& info) {
    if (!enabled_) return;
    uint32_t head = head_.load(std::memory_order_relaxed);
    uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= static_cast<uint32_t>(kRing)) {
        // writer behind: drop this row and count it — never wait, and never
        // touch tail_ (the writer owns it), so no row is torn mid-write
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    float* row = ring_.data() + static_cast<size_t>(head % kRing) * num_columns();
    fill(info, row);
    head_.store(head + 1, std::memory_order_release);
}

void StateTrace::fill(const TickInfo& info, float* row) {
    const size_t ncol = num_columns();
    std::fill(row, row + ncol, 0.0f);
    size_t i = 0;
    auto put  = [&](double v) { row[i++] = static_cast<float>(v); };
    auto skip = [&](int k) { i += static_cast<size_t>(k); };

    // tick
    put(std::chrono::duration<double>(info.t0 - start_time_).count());
    put(static_cast<double>(tick_++));
    put(info.wbc_state);
    put(static_cast<int>(ControlArbiter::instance().mode()));
    put(info.encoder_mode);
    put(info.planner_cursor);
    put(static_cast<double>(info.timing_enc_us));
    put(static_cast<double>(info.timing_dec_us));
    put(static_cast<double>(info.timing_total_us));
    put(static_cast<double>(dropped_.load(std::memory_order_relaxed)));

    // smpl
    if (auto w = TeleopTracker::instance().smpl_window_buf.GetData()) {
        const SmplPose& p = w->frames[SmplPoseWindow::kFrames - 1];
        put(1);
        for (const auto& j : p.joints_local) for (double v : j) put(v);
        for (double v : p.anchor_quat) put(v);
        for (double v : p.wrist_joint_pos) put(v);
    } else {
        put(0); skip(72 + 4 + 6);
    }

    // vr3point
    if (auto v = TeleopTracker::instance().vr3point_buf.GetData()) {
        put(1);
        for (double x : v->position) put(x);
        for (double x : v->orientation) put(x);
    } else {
        put(0); skip(9 + 12);
    }

    // planner frame
    if (info.planner_frame) {
        put(1);
        for (double x : info.planner_frame->joints) put(x);
    } else {
        put(0); skip(29);
    }

    // token
    if (auto tok = WholeBodyController::instance().motion_token_buf.GetData()) {
        put(1);
        for (float x : tok->token) put(x);
    } else {
        put(0); skip(64);
    }

    // decoder: action (IsaacLab order, raw) and published q_target (MuJoCo order)
    {
        const auto& a = WholeBodyController::instance().last_action();
        for (double x : a) put(x);
        if (auto cmd = WholeBodyController::instance().motor_command_buf.GetData()) {
            for (float x : cmd->q_target) put(x);
        } else {
            skip(29);
        }
    }

    // robot
    if (auto st = UnitreeStateReader::instance().unitree_state_buf.GetData()) {
        put(1);
        for (const auto& m : st->motors) put(m.q);
        for (const auto& m : st->motors) put(m.dq);
        for (const auto& m : st->motors) put(m.tau);
        for (const auto& m : st->motors) put(m.temp_casing);
        for (const auto& m : st->motors) put(m.temp_winding);
        for (const auto& m : st->motors) put(static_cast<double>(m.error));
        for (double x : st->imu_pelvis.quaternion) put(x);
        for (double x : st->imu_pelvis.gyroscope) put(x);
    } else {
        put(0); skip(29 * 6 + 4 + 3);
    }

    // health flags
    for (int m = 0; m < 29; ++m) put(MotorHealthMonitor::instance().flags(m));

    // hands
    {
        auto& hs = HandStateReader::instance();
        auto l = hs.left_buf.GetData();
        auto r = hs.right_buf.GetData();
        put(l ? 1 : 0); put(r ? 1 : 0);
        if (l) for (const auto& m : l->motors) put(m.q); else skip(7);
        if (r) for (const auto& m : r->motors) put(m.q); else skip(7);
        if (auto c = HandCommandWriter::instance().last_cmd_buf.GetData()) {
            for (float x : c->left.q) put(x);
            for (float x : c->right.q) put(x);
        } else {
            skip(14);
        }
    }

    // operator controller
    if (auto c = PicoVRReader::instance().ctrl_buf.GetData()) {
        put(1);
        put(c->left_axis[0]); put(c->left_axis[1]); put(c->right_axis[0]); put(c->right_axis[1]);
        put(c->btn_a); put(c->btn_b); put(c->btn_x); put(c->btn_y);
        put(c->left_trigger); put(c->right_trigger); put(c->left_grip); put(c->right_grip);
    } else {
        put(0); skip(12);
    }

    if (i != ncol)
        std::cerr << "[StateTrace] row layout mismatch: filled " << i << " of " << ncol << "\n";
}

} // namespace kist
