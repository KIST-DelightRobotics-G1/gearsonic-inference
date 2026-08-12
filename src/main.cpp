// Main deployment entry — THE ROBOT WILL MOVE on launch.
//
// All wiring and status live in GearsonicInference; this binary only runs
// the lifecycle: start -> wait for Ctrl+C -> damping shutdown.
//
// Operator controls (see README):
//   B held 1s      : teleop on (calibrate at reference pose) / off
//   A+B+X+Y held 1s: emergency stop (latched, damping)

#include "system/gearsonic_inference.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    auto& sys = kist::GearsonicInference::instance();
    sys.install_signal_handlers();

    std::cout << "*** ROBOT WILL MOVE: 3s ramp to standing, then policy control ***\n";
    if (!sys.start(config_path))
        return 1;

    while (!sys.quit_requested())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\nStopping: damping...\n";
    sys.stop();
    return 0;
}
