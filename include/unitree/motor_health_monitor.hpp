#pragma once

#include "common/data_buffer.hpp"
#include "control/motor_command.hpp"
#include "unitree/motor_health_rules.hpp"
#include "unitree/unitree_state.hpp"

#include <atomic>
#include <thread>

namespace kist {

// 20Hz observer over UnitreeStateReader / HandStateReader / the WBC command
// buffer. Runs MotorHealthRules and prints its lines to stderr (which the
// ConsoleTee mirrors into logs/latest.log). Pure observer: it commands
// nothing — what the robot should DO on a fault is decided elsewhere,
// once we know which faults occur in practice.
class MotorHealthMonitor {
public:
    static MotorHealthMonitor& instance();

    // body/cmd buffers are read each tick; both must outlive the monitor.
    bool start(const DataBuffer<UnitreeState>* body, const DataBuffer<MotorCommand>* cmd,
               const MotorHealthRules::Params& params = MotorHealthRules::Params{});
    void stop();

private:
    MotorHealthMonitor() = default;
    void loop();

    const DataBuffer<UnitreeState>* body_{nullptr};
    const DataBuffer<MotorCommand>* cmd_{nullptr};
    MotorHealthRules  rules_;
    std::thread       thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
