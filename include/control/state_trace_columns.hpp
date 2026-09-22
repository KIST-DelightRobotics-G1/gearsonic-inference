#pragma once

#include <string>
#include <vector>

namespace kist {

// The trace row layout, in order. Header-only so a probe can check it
// without the controller's TensorRT includes. Keep in step with
// StateTrace::fill().
inline std::vector<std::string> state_trace_columns() {
    std::vector<std::string> c;
    auto add  = [&](const std::string& n) { c.push_back(n); };
    auto addn = [&](const std::string& n, int k) { for (int i = 0; i < k; ++i) add(n + "_" + std::to_string(i)); };

    // tick
    add("t_s"); add("tick"); add("wbc_state"); add("arb_mode"); add("enc_mode");
    add("planner_cursor"); add("timing_enc_us"); add("timing_dec_us"); add("timing_total_us"); add("trace_dropped");
    // smpl (full-body reference, newest window frame)
    add("smpl_valid"); addn("smpl_j", 72); addn("smpl_anchor", 4); addn("smpl_wrist", 6);
    // vr3point (upper-body reference)
    add("vr3_valid"); addn("vr3_pos", 9); addn("vr3_orn", 12);
    // planner (current playback frame, MuJoCo order)
    add("planner_valid"); addn("planner_q", 29);
    // encoder token
    add("token_valid"); addn("token", 64);
    // decoder
    addn("action", 29); addn("q_target", 29);
    // robot
    add("state_valid"); addn("q", 29); addn("dq", 29); addn("tau", 29);
    addn("temp_casing", 29); addn("temp_winding", 29); addn("fault", 29);
    addn("imu_quat", 4); addn("imu_gyro", 3);
    // motor health flags (1 fault, 2 frozen, 4 not-responding)
    addn("health", 29);
    // hands
    add("hand_l_valid"); add("hand_r_valid");
    addn("hand_l_q", 7); addn("hand_r_q", 7); addn("hand_l_cmd", 7); addn("hand_r_cmd", 7);
    // operator controller
    add("ctrl_valid"); addn("ctrl_laxis", 2); addn("ctrl_raxis", 2);
    add("btn_a"); add("btn_b"); add("btn_x"); add("btn_y");
    add("trig_l"); add("trig_r"); add("grip_l"); add("grip_r");
    return c;
}


} // namespace kist
