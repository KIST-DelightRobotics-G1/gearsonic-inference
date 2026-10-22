#include "collector/record_commander.hpp"
#include "motion/input_handler.hpp"

#include <chrono>
#include <iostream>

namespace kist {

RecordCommander& RecordCommander::instance() {
    static RecordCommander inst;
    return inst;
}

bool RecordCommander::start() {
    try {
        pub_.reset(new CmdPub(kCmdTopic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[RecordControl] DDS publish init failed: " << e.what() << "\n";
        pub_.reset();
        return false;
    }
    assumed_ = RecordState::Idle;
    started_ = true;
    InputHandler::instance().set_tap_sink(
        [this](TapButton b, TapKind k) { on_tap(b, k); });
    std::cout << "[RecordControl] started (B tap start / B double stop / A tap success / "
                 "A double fail / A triple partial -> " << kCmdTopic << ")\n";
    return true;
}

void RecordCommander::stop() {
    started_ = false;
    InputHandler::instance().set_tap_sink(nullptr);
    pub_.reset();
}

void RecordCommander::on_tap(TapButton button, TapKind kind) {
    if (!started_) return;
    RecordState  st     = assumed_.load();
    RecordAction action = record_decide(st, button, kind);
    const char* bname = button == TapButton::A ? "A" : "B";
    const char* kname = kind == TapKind::Single ? "tap" : (kind == TapKind::Double ? "double" : "triple");
    if (action == RecordAction::None) {
        std::cout << "[RecordControl] " << bname << " " << kname << " ignored (state "
                  << record_state_name(st) << ")\n";
        return;
    }
    publish(action);
    assumed_ = record_next_state(st, action);
    std::cout << "[RecordControl] " << bname << " " << kname << " -> " << record_action_name(action)
              << " (" << record_state_name(st) << " -> " << record_state_name(assumed_.load()) << ")\n";
}

void RecordCommander::publish(RecordAction action) {
    std::lock_guard<std::mutex> lock(pub_mutex_);
    if (!pub_) return;
    kist_msgs::RecordCommand msg;
    msg.seq(++seq_);
    msg.stamp_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::system_clock::now().time_since_epoch()).count());
    msg.action(static_cast<uint8_t>(action));
    pub_->Write(msg);
}

} // namespace kist
