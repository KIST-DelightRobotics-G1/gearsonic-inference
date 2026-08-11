# kist-gearsonic-inference

C++ inference pipeline for GR00T WholeBodyControl on the Unitree G1 humanoid robot.

## Architecture

[![Architecture](docs/kist-gearsonic-inference.svg)](docs/kist-gearsonic-inference.svg)

## Dependencies

| Package | Purpose |
|---|---|
| `XRoboToolkit` | PICO VR PC daemon |
| `libPXREARobotSDK` | PICO VR C++ client library |
| `unitree_sdk2` | Unitree G1 + Dex3-1 hand DDS client library |
| `yaml-cpp` | YAML config parser |
| `CUDA == 12.6` | GPU runtime for TensorRT inference |
| `TensorRT == 10.7` | ONNX → TRT engine conversion and inference |

## Installation

### Clone Repository
```bash
git clone https://github.com/Safety-Node/kist-gearsonic-inference.git
cd kist-gearsonic-inference
```

All following steps run from the repository root.

### Quick Start with Docker (recommended)

The image is self-contained: it clones the pinned dependencies
(unitree_sdk2, the PXREA VR client SDK), installs the CycloneDDS idlc
toolchain, downloads the GEAR-SONIC models, bakes the source in, and builds
the binaries — all at image-build time.

```bash
./docker/build.sh      # builds everything into the image
./docker/run.sh        # shell with ready binaries under build/
```

The only host-side requirement besides Docker (with the NVIDIA runtime) is
the **XRoboToolkit PC service** below — the VR daemon talks to the headset
over USB and runs on the host; the container reaches it via `--network
host`. Everything from *Install libPXREARobotSDK* on is the manual
(non-Docker) alternative.

### Install XRoboToolkit (host-side, required either way)

```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

### Install libPXREARobotSDK

```bash
mkdir -p thirdparty/pxrea/lib thirdparty/pxrea/include
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git thirdparty/XRoboToolkit-PC-Service

cd thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK
bash build.sh
cd ../../../..

cp thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h thirdparty/pxrea/include/
cp -r thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann thirdparty/pxrea/include/nlohmann/
cp thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so thirdparty/pxrea/lib/
```

### Install unitree_sdk2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2
```

### Download Models

```bash
wget -P models https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/model_encoder.onnx
wget -P models https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/model_decoder.onnx
wget -P models https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/planner_sonic.onnx
```

ONNX models are converted to TensorRT engines automatically on first run.

### Install CycloneDDS (idlc toolchain)

Required for the VLA DDS types (`idl/kist_latent_action.idl` -> C++ at build
time). Version pinned to 0.10.2 to match unitree_sdk2's bundled ddscxx --
same setup as kist-ext-sensor-io. Included in the Docker image.

```bash
git clone --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds
cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
    -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF -DENABLE_SSL=OFF -DENABLE_SECURITY=OFF
sudo cmake --build /tmp/cyclonedds/build -j --target install
git clone --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git /tmp/cyclonedds-cxx
cmake -S /tmp/cyclonedds-cxx -B /tmp/cyclonedds-cxx/build \
    -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DCMAKE_PREFIX_PATH=/opt/cyclonedds \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
sudo cmake --build /tmp/cyclonedds-cxx/build -j --target install
```

### Install yaml-cpp

```bash
sudo apt install libyaml-cpp-dev
```

### Install CUDA and TensorRT

CUDA 12.6 and TensorRT 10.7, per the NVIDIA guides:

- https://developer.nvidia.com/cuda-12-6-0-download-archive
- https://developer.nvidia.com/tensorrt/download/10x

## Build

With Docker, run this inside the container (`./docker/run.sh`).

```bash
cmake -B build && cmake --build build
```

## Run

### 1. XRoboToolkit (PICO VR daemon)

With Docker, run this on the host, outside the container.

```bash
source env.sh
run_vr_daemon
```

Connect the headset from its XRoboToolkit app.

### 2. Control

```bash
./build/gearsonic_inference
```

### Controller

| Input | Action |
|---|---|
| Left stick | Move (magnitude = speed) |
| Right stick | Rotate facing |
| A | Return to IDLE |
| Y | Mode up (IDLE / Slow Walk / Walk) |
| Trigger + Y | Mode up (hard actions, e.g. Run) |
| X | Mode down |
| Trigger + B / A | Height up / down (crouch modes) |
| B held 1s | Teleop on / off (engage in the reference pose: forearms 90° forward, palms inward) |
| Left / right grip (analog) | Left / right Dex3-1 thumb close (0 = open, 1 = pressed against fingers) |
| Left / right trigger (analog) | Left / right Dex3-1 index+middle close (0 = open, 1 = cage / fist) |
| A + B + X + Y held 1s | Emergency stop |

## Usage

Embedding as a C++ library:

```cmake
add_subdirectory(kist-gearsonic-inference)
target_link_libraries(your_app PRIVATE gearsonic_inference)
```

```cpp
#include "system/gearsonic_inference.hpp"
#include "motion/input_handler.hpp"

auto& gearsonic_inf = kist::GearsonicInference::instance();
gearsonic_inf.install_signal_handlers();          // or call gearsonic_inf.request_quit() from your own handler

if (!gearsonic_inf.start("config/config.yaml"))   // THE ROBOT MOVES: 3s ramp, then policy control
    return 1;

// external navigation (optional): body-frame velocity, ~20Hz.
// zeros = stop, going silent = fallback to manual. Joystick always wins.
kist::InputHandler::instance().nav_buf.SetData({vx, vy, vyaw});

// ... your application runs here (keep the process alive) ...

gearsonic_inf.stop();                             // publishes damping — call on every exit path
```

## VLA mode (external latent tokens over DDS)

[kist-vla-inference](https://github.com/Safety-Node/kist-vla-inference) can
drive the whole body directly: it publishes 64-dim SONIC motion tokens +
Dex3 hand targets as `kist_msgs::LatentActionStep` on `rt/kist/latent_action`
(50 Hz), replacing the planner+encoder path.

- **Contract**: `idl/kist_latent_action.idl` — shared with kist-vla-inference;
  built into C++ types at build time by CycloneDDS `idlc` (0.10.2, matching
  the SDK's bundled ddscxx — a required dependency, see Install CycloneDDS;
  the Docker image includes it).
- **Behavior**: first-come ownership (`include/control/robot_ownership.hpp`)
  arbitrates VLA vs teleop — whichever claims the robot first keeps it.
  A token fresher than 200 ms claims for VLA and switches
  `WholeBodyController` into external-token mode (planner path bypassed);
  if teleop calibrated first, VLA tokens are ignored until restart.
  VLA ownership is sticky — a stream stale for 500 ms counts as LOST and
  the controller blends (1 s) to a verified safe standing token and keeps
  balancing there, hands holding their last targets; neither a resumed
  stream nor the planner path (its playback timeline froze at the pre-VLA
  state) may retake the robot — restart to recover. The e-stop outranks
  everything, always.
- **Interop probe**: `./build/vla_receiver_probe [domain_id] [iface]`
  prints receive rate and newest token — pair it with kist-vla-inference
  in `--io.action-transport dds` mode to verify the link without the robot.
