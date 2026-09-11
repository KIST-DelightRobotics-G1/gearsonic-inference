#include "unitree/motor_health_monitor.hpp"
#include "unitree/hand_state_reader.hpp"

#include <chrono>
#include <iostream>

namespace kist {

static constexpr auto   kPeriod       = std::chrono::milliseconds(50);  // 20 Hz
static constexpr double kHandStaleMs  = 200.0;

MotorHealthMonitor& MotorHealthMonitor::instance() {
    static MotorHealthMonitor inst;
    return inst;
}

bool MotorHealthMonitor::start(const DataBuffer<UnitreeState>* body, const DataBuffer<MotorCommand>* cmd,
                               const MotorHealthRules::Params& params) {
    body_ = body;
    cmd_  = cmd;
    rules_.configure(params);
    if (!params.enabled) {
        std::cout << "[MotorHealthMonitor] disabled by config\n";
        return true;
    }
    stop_   = false;
    thread_ = std::thread(&MotorHealthMonitor::loop, this);
    std::cout << "[MotorHealthMonitor] started (20 Hz; casing warn/critical " << params.casing_warn_c
              << "/" << params.casing_critical_c << "C, winding " << params.winding_warn_c
              << "/" << params.winding_critical_c << "C; not-responding tau_est<"
              << params.unresp_tau_ratio << "x expected (>" << params.unresp_tau_min << " Nm) for " << params.unresp_time_s
              << "s; frozen " << params.frozen_time_s << "s; log-only)\n";
    return true;
}

void MotorHealthMonitor::stop() {
    stop_ = true;
    if (thread_.joinable())
        thread_.join();
}

void MotorHealthMonitor::loop() {
    using clock = std::chrono::steady_clock;
    auto last = clock::now();
    while (!stop_) {
        std::this_thread::sleep_for(kPeriod);
        auto now  = clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        auto body = body_ ? body_->GetData() : nullptr;
        auto cmd  = cmd_  ? cmd_->GetData()  : nullptr;
        auto& hands = HandStateReader::instance();
        auto ls = hands.left_buf.GetDataWithTime();
        auto rs = hands.right_buf.GetDataWithTime();
        const HandState* l = (ls.HasData() && ls.GetAgeMs() < kHandStaleMs) ? ls.data.get() : nullptr;
        const HandState* r = (rs.HasData() && rs.GetAgeMs() < kHandStaleMs) ? rs.data.get() : nullptr;

        for (const auto& line : rules_.step(body.get(), cmd.get(), l, r, dt))
            std::cerr << line << "\n";
    }
}

} // namespace kist
