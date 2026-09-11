#include "unitree/unitree_state_reader.hpp"
#include "unitree/crc32.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <chrono>
#include <iostream>

namespace kist {

static const std::string kLowStateTopic = "rt/lowstate";
static const std::string kImuTorsoTopic = "rt/secondary_imu";

// ─── SDK → kist conversion helpers ────────────────────────────────────────────

static IMU convert(const unitree_hg::msg::dds_::IMUState_& src) {
    IMU out;
    const auto& q = src.quaternion();
    const auto& g = src.gyroscope();
    const auto& a = src.accelerometer();
    out.quaternion    = { q[0], q[1], q[2], q[3] };
    out.gyroscope     = { g[0], g[1], g[2] };
    out.accelerometer = { a[0], a[1], a[2] };
    return out;
}

static UnitreeState convert(const unitree_hg::msg::dds_::LowState_& src) {
    UnitreeState out;
    out.tick = src.tick();
    out.mode_machine = src.mode_machine();
    out.imu_pelvis = convert(src.imu_state());
    const auto& motors = src.motor_state();
    for (int i = 0; i < kNumMotors; ++i) {
        out.motors[i].q   = motors[i].q();
        out.motors[i].dq  = motors[i].dq();
        out.motors[i].tau = motors[i].tau_est();
        out.motors[i].temp_casing  = motors[i].temperature()[0];
        out.motors[i].temp_winding = motors[i].temperature()[1];
        out.motors[i].error       = motors[i].motorstate();
    }
    return out;
}

// ─── UnitreeStateReader ───────────────────────────────────────────────────────

UnitreeStateReader& UnitreeStateReader::instance() {
    static UnitreeStateReader inst;
    return inst;
}

bool UnitreeStateReader::start(int domain_id, const std::string& network_interface) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        lowstate_sub_.reset(new LowStateSub(kLowStateTopic));
        lowstate_sub_->InitChannel(
            [this](const void* msg) { on_lowstate_update(msg); }, 1);

        imu_torso_sub_.reset(new IMUStateSub(kImuTorsoTopic));
        imu_torso_sub_->InitChannel(
            [this](const void* msg) { on_imu_torso_update(msg); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[UnitreeStateReader] DDS init failed on interface \""
                  << network_interface << "\": " << e.what()
                  << "\n  Check the robot LAN cable and config unitree.network_interface.\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&UnitreeStateReader::watchdog_loop, this);
    std::cout << "[UnitreeStateReader] started on domain=" << domain_id
              << " interface=" << network_interface << "\n";
    return true;
}

void UnitreeStateReader::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    lowstate_sub_.reset();
    imu_torso_sub_.reset();
}

void UnitreeStateReader::on_lowstate_update(const void* message) {
    const auto& low_state = *static_cast<const SdkLowState*>(message);

    // Discard corrupted packets: this state feeds policy observations and
    // the joint-velocity safety check directly.
    uint32_t crc = crc32_core(reinterpret_cast<const uint32_t*>(&low_state),
                              (sizeof(SdkLowState) >> 2) - 1);
    if (crc != low_state.crc()) {
        std::cerr << "[UnitreeStateReader] LowState CRC mismatch — packet dropped\n";
        return;
    }

    unitree_state_buf.SetData(convert(low_state));
}

void UnitreeStateReader::on_imu_torso_update(const void* message) {
    imu_torso_buf.SetData(convert(*static_cast<const SdkIMUState*>(message)));
}

// Streams are watched independently. Clearing at 60ms (30 frames at 500Hz)
// is a deliberate tightening of gear_sonic's 500ms LowState-absence limit:
// an empty buffer fails the control loop's safety check immediately, so a
// genuine link loss reaches damping after 60ms of blind control instead of
// 500ms. The accepted trade-off is that a transient DDS hiccup past 60ms
// causes a spurious safe-stop — visible in the log if it ever happens.
void UnitreeStateReader::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 60.0;  // 30 frames at 500Hz

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto state = unitree_state_buf.GetDataWithTime();
        if (state.HasData() && state.GetAgeMs() > stale_ms) {
            std::cerr << "[UnitreeStateReader] LowState stale — cleared\n";
            unitree_state_buf.Clear();
        }

        auto imu = imu_torso_buf.GetDataWithTime();
        if (imu.HasData() && imu.GetAgeMs() > stale_ms) {
            std::cerr << "[UnitreeStateReader] torso IMU stale — cleared\n";
            imu_torso_buf.Clear();
        }
    }
}

} // namespace kist
