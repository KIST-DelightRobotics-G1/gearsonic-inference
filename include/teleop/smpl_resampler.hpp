#pragma once

#include "common/math_utils.hpp"
#include "teleop/smpl_pose.hpp"

#include <algorithm>
#include <array>
#include <chrono>

namespace kist {

// Re-samples the irregular headset body stream onto the tracker's 50Hz
// grid, the way gear_sonic's pico server does before streaming (linear on
// positions, slerp on the root orientation). The encoder's smpl mode reads
// ten frames as a 50Hz-spaced trajectory — frame differences are its
// reference velocities — so holding the newest sample into repeated ticks
// (stairs) or skipping samples (uneven spacing) feeds it velocities that
// alternate between zero and double, and the policy tracks that jitter.
//
// Grid values are taken `delay` behind now so both bracketing samples
// have arrived; the delay adapts to the measured inter-arrival interval
// (delay_factor x EMA, clamped), since the device carries no timestamp
// and its rate drifts with the WiFi link. Late data holds the newest
// sample (alpha = 1) instead of extrapolating.
//
// Pure: push() device samples with their receive time, sample() at grid
// times. Keeps a short ring of samples so the grid point is always
// bracketed even with the delay at 1.5-2 intervals.
class SmplResampler {
public:
    using clock = std::chrono::steady_clock;

    struct Params {
        double delay_factor{1.5};          // x measured interval
        double min_delay_s{0.020};
        double max_delay_s{0.100};
        double ema{0.1};                   // interval smoothing
        double seed_interval_s{1.0 / 30};  // before the first two samples
    };

    SmplResampler() = default;
    explicit SmplResampler(const Params& p) : p_(p) { reset(); }

    void reset() {
        count_ = 0;
        head_  = 0;
        interval_s_ = p_.seed_interval_s;
    }

    void push(const SmplPose& pose, clock::time_point t) {
        if (count_ > 0) {
            double dt = std::chrono::duration<double>(t - newest_t()).count();
            if (dt <= 0.0) return;   // duplicate / out-of-order receive: ignore
            interval_s_ = count_ == 1 ? dt : (1.0 - p_.ema) * interval_s_ + p_.ema * dt;
            head_ = (head_ + 1) % kRing;
        }
        poses_[head_] = pose;
        times_[head_] = t;
        if (count_ < kRing) ++count_;
    }

    // Value on the grid at `now - delay()`. False until the first sample.
    // The target is bracketed by the two ring samples around it; beyond the
    // newest (data late) the newest is held, before the oldest the oldest.
    bool sample(clock::time_point now, SmplPose& out) const {
        if (count_ == 0) return false;
        auto target = now - std::chrono::duration_cast<clock::duration>(
                                std::chrono::duration<double>(delay_s()));
        // walk from the newest back until a sample at or before the target
        int newer = head_;
        if (target >= times_[newer]) { out = poses_[newer]; return true; }
        for (int k = 1; k < count_; ++k) {
            int older = (head_ - k + kRing) % kRing;
            if (times_[older] <= target) {
                out = lerp(poses_[older], poses_[newer], alpha_for(times_[older], times_[newer], target));
                return true;
            }
            newer = older;
        }
        out = poses_[newer];   // older than everything we hold
        return true;
    }

    double interval_s() const { return interval_s_; }
    double delay_s() const {
        return std::clamp(p_.delay_factor * interval_s_, p_.min_delay_s, p_.max_delay_s);
    }
    int count() const { return count_; }
    clock::time_point newest_t() const { return times_[head_]; }

    // Position of `target` between t0 and t1, clamped to [0, 1]: past t1
    // (data late) holds the newest sample rather than extrapolating.
    static double alpha_for(clock::time_point t0, clock::time_point t1, clock::time_point target) {
        double span = std::chrono::duration<double>(t1 - t0).count();
        if (span <= 0.0) return 1.0;
        double a = std::chrono::duration<double>(target - t0).count() / span;
        return std::clamp(a, 0.0, 1.0);
    }

    static SmplPose lerp(const SmplPose& a, const SmplPose& b, double alpha) {
        SmplPose out;
        for (int j = 0; j < 24; ++j)
            for (int c = 0; c < 3; ++c)
                out.joints_local[j][c] = a.joints_local[j][c] + alpha * (b.joints_local[j][c] - a.joints_local[j][c]);
        for (int i = 0; i < 6; ++i)
            out.wrist_joint_pos[i] = a.wrist_joint_pos[i] + alpha * (b.wrist_joint_pos[i] - a.wrist_joint_pos[i]);
        out.anchor_quat = quat_slerp(a.anchor_quat, b.anchor_quat, alpha);
        return out;
    }

private:
    static constexpr int kRing = 8;   // 8 samples cover >= 260 ms at 30 Hz
    Params p_{};
    std::array<SmplPose, kRing>          poses_{};
    std::array<clock::time_point, kRing> times_{};
    int    head_{0};     // index of the newest sample
    int    count_{0};
    double interval_s_{1.0 / 30};
};

} // namespace kist
