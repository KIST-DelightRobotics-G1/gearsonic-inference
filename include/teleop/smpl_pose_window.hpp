#pragma once

#include "teleop/smpl_pose.hpp"

#include <array>

namespace kist {

// The ten 50Hz frames the encoder's smpl (full-body) mode reads, exactly as
// it should see them: frames[0] is the frame the policy tracks and
// frames[9] the furthest "future". Live tracking has no future, so
// TeleopTracker runs a delay line — the policy tracks the operator a few
// ticks late and the newest sample is held into the remaining slots (the
// same shape gear_sonic's streamed-motion merger produces). Built and
// published whole every tracker tick, so the reader never sees a
// half-shifted window; cleared when full-body teleop disengages.
struct SmplPoseWindow {
    static constexpr int kFrames = 10;
    std::array<SmplPose, kFrames> frames;
};

} // namespace kist
