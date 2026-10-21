#pragma once

#include <cstdint>

namespace kist {

// Recording-control vocabulary shared by the VR gesture side and the
// collector (wire values match idl/kist_msgs.idl RecordCommand).
enum class RecordAction : uint8_t { None = 0, Start = 1, Stop = 2, LabelSuccess = 3, LabelFail = 4 };
enum class RecordState  : uint8_t { Idle = 0, Recording = 1, AwaitingLabel = 2, Unknown = 255 };

enum class TapButton : uint8_t { A, B };
enum class TapKind   : uint8_t { Single, Double };

// Gesture -> command, given the collector's last reported state.
//   B single : start a recording (idle, or awaiting label -> previous
//              episode stays unlabeled)
//   B double : stop the recording
//   A single : label the stopped episode success (A alone also drops
//              locomotion to IDLE, as before — that is fine after a stop)
//   A double : label it fail
// The state passed in is the commander's local mirror of what it has
// sent (record_next_state), since the link is one-way: the collector's
// real state is read on its own console.
inline RecordAction record_decide(RecordState state, TapButton button, TapKind kind) {
    if (state == RecordState::Unknown) state = RecordState::Idle;
    switch (button) {
        case TapButton::B:
            if (kind == TapKind::Single)
                return state != RecordState::Recording ? RecordAction::Start : RecordAction::None;
            return state == RecordState::Recording ? RecordAction::Stop : RecordAction::None;
        case TapButton::A:
            if (state != RecordState::AwaitingLabel) return RecordAction::None;
            return kind == TapKind::Single ? RecordAction::LabelSuccess : RecordAction::LabelFail;
    }
    return RecordAction::None;
}

// What the collector will be in after it executes `action` — the
// commander's local mirror.
inline RecordState record_next_state(RecordState current, RecordAction action) {
    switch (action) {
        case RecordAction::Start:        return RecordState::Recording;
        case RecordAction::Stop:         return RecordState::AwaitingLabel;
        case RecordAction::LabelSuccess:
        case RecordAction::LabelFail:    return RecordState::Idle;
        default:                         return current;
    }
}

inline const char* record_action_name(RecordAction a) {
    switch (a) {
        case RecordAction::Start:        return "start";
        case RecordAction::Stop:         return "stop";
        case RecordAction::LabelSuccess: return "label_success";
        case RecordAction::LabelFail:    return "label_fail";
        default:                         return "none";
    }
}
inline const char* record_state_name(RecordState s) {
    switch (s) {
        case RecordState::Idle:          return "idle";
        case RecordState::Recording:     return "recording";
        case RecordState::AwaitingLabel: return "awaiting_label";
        default:                         return "unknown";
    }
}

} // namespace kist
