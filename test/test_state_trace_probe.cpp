// Checks the trace row layout: names unique, count equal to what
// StateTrace::fill() packs (sum of the group widths), and the anchor
// columns where a reader expects them. Header-only, no hardware.
//
// Run: ./build/test_state_trace_probe   (exit 0 = layout holds)

#include "control/state_trace_columns.hpp"

#include <cstdio>
#include <set>

static int failures = 0;
static void check(const char* what, bool ok) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", what);
    if (!ok) ++failures;
}

int main() {
    auto cols = kist::state_trace_columns();
    // group widths as fill() packs them
    const size_t expect = 10                     // tick
                        + 1 + 72 + 4 + 6         // smpl
                        + 1 + 9 + 12             // vr3
                        + 1 + 29                 // planner
                        + 1 + 64                 // token
                        + 29 + 29                // action, q_target
                        + 1 + 29 * 6 + 4 + 3     // robot + imu
                        + 29                     // health
                        + 2 + 7 * 4              // hands
                        + 1 + 12;                // controller
    check("column count matches fill()", cols.size() == expect);
    std::set<std::string> names(cols.begin(), cols.end());
    check("column names unique", names.size() == cols.size());
    check("first column is t_s", cols.front() == "t_s");
    check("last column is grip_r", cols.back() == "grip_r");
    check("q_0 present", names.count("q_0") == 1);
    check("token_63 present", names.count("token_63") == 1);
    check("smpl_j_71 present", names.count("smpl_j_71") == 1);
    std::printf("%zu columns -> %.1f KB per row at float32\n", cols.size(), cols.size() * 4 / 1024.0);
    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
