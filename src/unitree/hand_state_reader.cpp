#include "unitree/hand_state_reader.hpp"

#include <algorithm>
#include <iostream>
#include <string>

namespace kist {

static const std::string kLeftStateTopic  = "rt/dex3/left/state";
static const std::string kRightStateTopic = "rt/dex3/right/state";

// The hand controller reports motor_state as a vector; a short packet
// leaves the missing motors at zero rather than reading out of range.
static HandState convert(const unitree_hg::msg::dds_::HandState_& src) {
    HandState out;
    const auto& motors = src.motor_state();
    const int n = std::min<int>(kHandMotors, static_cast<int>(motors.size()));
    for (int i = 0; i < n; ++i) {
        const auto& m = motors[i];
        out.motors[i].q           = m.q();
        out.motors[i].dq          = m.dq();
        out.motors[i].tau         = m.tau_est();
        out.motors[i].temperature = std::max(m.temperature()[0], m.temperature()[1]);
    }
    out.power_a = src.power_a();
    out.error   = src.error()[0];
    return out;
}

HandStateReader& HandStateReader::instance() {
    static HandStateReader inst;
    return inst;
}

bool HandStateReader::start() {
    // ChannelFactory must already be initialized by UnitreeStateReader::start().
    try {
        left_sub_.reset(new HandStateSub(kLeftStateTopic));
        left_sub_->InitChannel([this](const void* msg) { on_left_update(msg); }, 1);

        right_sub_.reset(new HandStateSub(kRightStateTopic));
        right_sub_->InitChannel([this](const void* msg) { on_right_update(msg); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[HandStateReader] DDS subscribe failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[HandStateReader] started (subscribed to " << kLeftStateTopic
              << " and " << kRightStateTopic << ")\n";
    return true;
}

void HandStateReader::stop() {
    left_sub_.reset();
    right_sub_.reset();
    left_buf.Clear();
    right_buf.Clear();
    std::cout << "[HandStateReader] stopped\n";
}

void HandStateReader::on_left_update(const void* message) {
    left_buf.SetData(convert(*static_cast<const SdkHandState*>(message)));
}

void HandStateReader::on_right_update(const void* message) {
    right_buf.SetData(convert(*static_cast<const SdkHandState*>(message)));
}

} // namespace kist
