# kist-gearsonic-inference

C++ inference pipeline for GR00T WholeBodyControl on the Unitree G1 humanoid robot.

## Architecture

[![Architecture](docs/kist-gearsonic-inference.svg)](docs/kist-gearsonic-inference.svg)

## Dependencies

| Component | Version | Role |
|---|---|---|
| `XRoboToolkit` | 1.0.0 | PICO VR PC daemon (host-side) |
| `libPXREARobotSDK` | `85bac4d` | PICO VR C++ client library |
| `unitree_sdk2` | `21d0a3b` | Unitree G1 + Dex3-1 hand DDS client library |
| CycloneDDS + CycloneDDS-CXX | 0.10.2 | `idlc`/`idlcxx` codegen for the VLA DDS types |
| CUDA | 12.6 | GPU runtime for TensorRT inference |
| TensorRT | 10.7 | ONNX → TRT engine conversion and inference |
| `yaml-cpp` | distro | config parsing |
| GEAR-SONIC models | HF `nvidia/GEAR-SONIC` | planner / encoder / decoder ONNX |

## Installation

#### 0. Onboard Orin (Jetson) only

Running gearsonic on the onboard Orin needs one-time JetPack/docker setup
before anything below — see [docs/jetson_setup.md](docs/jetson_setup.md).
Skip on an x86_64 workstation.

#### 1. Clone Repository

```bash
git clone https://github.com/Safety-Node/kist-gearsonic-inference.git
cd kist-gearsonic-inference
```

All following steps run from the repository root.

#### 2. Install XRoboToolkit (host-side)

The VR daemon talks to the headset and runs on the host; the container
reaches it via `--network host`. Pick the package for the host arch:

**x86_64 workstation:**

```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

**aarch64 onboard (Jetson / Orin):** nothing to install on the host — the
daemon is baked into the image. See [docs/jetson_setup.md](docs/jetson_setup.md).

#### Quick Start with Docker

The image bakes in everything below (SDKs, toolchain, models, and the build):

```bash
./docker/build.sh      # builds the image (docker build -t kist-gearsonic-inference)
./docker/run.sh        # shell in the container; prebuilt binaries under build/
```

`build.sh` auto-selects the Dockerfile by architecture: `docker/Dockerfile`
on x86_64 (workstation), `docker/Dockerfile.aarch64` on the Jetson / onboard
Orin (running gearsonic onboard removes the PC↔robot link entirely). Same
image tag, so `run.sh` is identical on both.

`run.sh` wires `--gpus all` (TensorRT), `--network host` (unitree/VLA DDS +
the VR daemon), and `--cap-add=SYS_NICE` (RT thread priorities). The numbered
steps below (3–8) are the manual (non-Docker) alternative — steps 1–2
above are required either way.

#### 3. Install libPXREARobotSDK

```bash
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git thirdparty/XRoboToolkit-PC-Service
git -C thirdparty/XRoboToolkit-PC-Service checkout 85bac4dbc1fd5cef42c74a160d9c30aa3491f122

bash thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build.sh

mkdir -p thirdparty/pxrea/lib thirdparty/pxrea/include
cp thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h thirdparty/pxrea/include/
cp -r thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann thirdparty/pxrea/include/nlohmann/
cp thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so thirdparty/pxrea/lib/
```

#### 4. Install unitree_sdk2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2
git -C thirdparty/unitree_sdk2 checkout 21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
```

#### 5. Install apt packages

```bash
sudo apt update && sudo apt install -y \
    build-essential cmake git pkg-config \
    libyaml-cpp-dev libssl-dev
```

#### 6. Install CycloneDDS (idlc toolchain)

CycloneDDS + CycloneDDS-CXX 0.10.2 into `/opt/cyclonedds`, pinned to match the
SDK's bundled `libddscxx` (required — the VLA DDS types are generated from
`idl/kist_msgs.idl` at build time):

```bash
git clone --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds
cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
    -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DBUILD_IDLC=ON -DCMAKE_BUILD_TYPE=Release
sudo cmake --build /tmp/cyclonedds/build --target install -j"$(nproc)"

git clone --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git /tmp/cyclonedds-cxx
cmake -S /tmp/cyclonedds-cxx -B /tmp/cyclonedds-cxx/build \
    -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DCMAKE_PREFIX_PATH=/opt/cyclonedds -DCMAKE_BUILD_TYPE=Release
sudo cmake --build /tmp/cyclonedds-cxx/build --target install -j"$(nproc)"

export PATH=/opt/cyclonedds/bin:$PATH      # idlc on PATH for Build
```

#### 7. Install CUDA and TensorRT

CUDA 12.6 and TensorRT 10.7, per the NVIDIA guides:

- https://developer.nvidia.com/cuda-12-6-0-download-archive
- https://developer.nvidia.com/tensorrt/download/10x

#### 8. Download Models

```bash
wget -P models https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/model_encoder.onnx
wget -P models https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/model_decoder.onnx
wget -P models https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/planner_sonic.onnx
```

ONNX models are converted to TensorRT engines automatically on first run.

## Build

With Docker, run this inside the container (`./docker/run.sh`).

```bash
cmake -B build && cmake --build build
```

## Usage

Set up the config once before running:

- `config/config.yaml` — robot NIC (`unitree.network_interface`), DDS
  domain, model paths.
- All keys: [docs/configuration.md](docs/configuration.md).

**THE ROBOT MOVES on launch** — hang it or clear the area, keep the VR
controller in reach (A+B+X+Y held 1s = emergency stop).

```bash
# host: VR daemon (required in every mode — it carries the e-stop)
source env.sh && run_vr_daemon

# container: control (3s ramp to standing, then policy control)
./build/kist-gearsonic-inference
```

Both modes run from the same binary; ownership is first-come:

- **Teleop mode** — joystick locomotion, VR arm/hand tracking:
  [docs/teleop_mode.md](docs/teleop_mode.md)
- **VLA mode** — external latent tokens from kist-vla-inference over DDS:
  [docs/vla_mode.md](docs/vla_mode.md)

> Embedding as a C++ library: [docs/embedding.md](docs/embedding.md)
