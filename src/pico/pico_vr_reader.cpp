#include "pico/pico_vr_reader.hpp"

#include <PXREARobotSDK.h>
#include <nlohmann/json.hpp>

#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using json = nlohmann::json;

namespace kist {

// ─── helpers ──────────────────────────────────────────────────────────────────

static std::array<double, 7> parse_pose(const std::string& s) {
    std::array<double, 7> result{};
    std::stringstream ss(s);
    std::string token;
    int i = 0;
    while (std::getline(ss, token, ',') && i < 7)
        result[i++] = std::stod(token);
    return result;
}

// ─── SDK callback ─────────────────────────────────────────────────────────────

static void pxrea_callback(void*, PXREAClientCallbackType type, int, void* user_data) {
    auto& reader = PicoVRReader::instance();

    if (type != PXREADeviceStateJson)
        return;

    auto& dsj = *static_cast<PXREADevStateJson*>(user_data);

    try {
        auto data = json::parse(dsj.stateJson);
        if (!data.contains("value"))
            return;

        auto value = json::parse(data["value"].get<std::string>());

        // ── Body tracking ──────────────────────────────────────────────────
        if (value.contains("Body")) {
            auto& body = value["Body"];
            if (body.contains("joints") && body["joints"].is_array()) {
                PicoVRBodyPose pose;
                auto& joints = body["joints"];
                int n = std::min(static_cast<int>(joints.size()), 24);
                for (int i = 0; i < n; i++) {
                    if (joints[i].contains("p"))
                        pose.joints[i] = parse_pose(joints[i]["p"].get<std::string>());
                }
                reader.on_body_update(pose);
            }
        }

        // ── Controller ────────────────────────────────────────────────────
        PicoVRController ctrl;
        bool ctrl_updated = false;

        if (value["Controller"].contains("left")) {
            auto& l = value["Controller"]["left"];
            ctrl.left_axis[0] = l.value("axisX", 0.0);
            ctrl.left_axis[1] = l.value("axisY", 0.0);
            ctrl.left_trigger = l.value("trigger", 0.0);
            ctrl.left_grip    = l.value("grip", 0.0);
            ctrl.btn_x        = l.value("primaryButton", false);
            ctrl.btn_y        = l.value("secondaryButton", false);
            ctrl_updated = true;
        }
        if (value["Controller"].contains("right")) {
            auto& r = value["Controller"]["right"];
            ctrl.right_axis[0] = r.value("axisX", 0.0);
            ctrl.right_axis[1] = r.value("axisY", 0.0);
            ctrl.right_trigger = r.value("trigger", 0.0);
            ctrl.right_grip    = r.value("grip", 0.0);
            ctrl.btn_a         = r.value("primaryButton", false);
            ctrl.btn_b         = r.value("secondaryButton", false);
            ctrl_updated = true;
        }
        if (ctrl_updated)
            reader.on_controller_update(ctrl);

    } catch (const json::exception& e) {
        std::cerr << "[PicoVRReader] JSON parse error: " << e.what() << "\n";
    }
}

// ─── RoboticsService daemon probe ─────────────────────────────────────────────
//
// The PXREA SDK is a gRPC client of the XRoboToolkit PC service
// (RoboticsServiceProcess) listening on 127.0.0.1:60061. The daemon is run
// manually on the host — see docs/teleop_mode.md. We only probe
// for it here: PXREAInit() succeeds even with no daemon (lazy gRPC channel),
// so the port check is the one reliable way to fail fast with a clear error.

static constexpr uint16_t kDaemonPort = 60061;

static bool daemon_port_open() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(kDaemonPort);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    // Blocking connect to localhost: refused/accepted both return immediately.
    bool open = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    close(fd);
    return open;
}

// ─── PicoVRReader ─────────────────────────────────────────────────────────────

PicoVRReader& PicoVRReader::instance() {
    static PicoVRReader inst;
    return inst;
}

bool PicoVRReader::start() {
    if (!daemon_port_open()) {
        std::cerr << "[PicoVRReader] XRoboToolkit service not running on port "
                  << kDaemonPort << " — start it on the host first"
                  << " (see docs/teleop_mode.md)\n";
        return false;
    }

    if (PXREAInit(nullptr, pxrea_callback, PXREAFullMask) != 0) {
        std::cerr << "[PicoVRReader] PXREAInit failed\n";
        return false;
    }
    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&PicoVRReader::watchdog_loop, this);
    std::cout << "[PicoVRReader] started (waiting for headset data)\n";
    return true;
}

void PicoVRReader::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    PXREADeinit();
}

void PicoVRReader::on_body_update(const PicoVRBodyPose& pose) {
    body_buf.SetData(pose);
}

void PicoVRReader::on_controller_update(const PicoVRController& ctrl) {
    ctrl_buf.SetData(ctrl);
}

// Each stream is watched independently: a dropped body-tracking stream must
// not clear fresh controller data (the control chain runs on ctrl alone),
// and a controller-only session must still clear its buffer when the link
// dies — InputHandler's safe-stop (IDLE) triggers off an empty ctrl_buf.
void PicoVRReader::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 145.0;  // 10 frames at 70Hz (worst case, low battery)

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(14ms);

        auto body = body_buf.GetDataWithTime();
        if (body.HasData() && body.GetAgeMs() > stale_ms) {
            std::cerr << "[PicoVRReader] body stream stale — cleared\n";
            body_buf.Clear();
        }

        auto ctrl = ctrl_buf.GetDataWithTime();
        if (ctrl.HasData() && ctrl.GetAgeMs() > stale_ms) {
            std::cerr << "[PicoVRReader] controller stream stale — cleared\n";
            ctrl_buf.Clear();
        }
    }
}

} // namespace kist
