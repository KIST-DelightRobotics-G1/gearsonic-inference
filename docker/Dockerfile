# x86_64 workstation container — the complete gearsonic build+run environment.
# TensorRT 10.7 + CUDA 12.6 to match the Jetson (JetPack 6 / TRT 10.7) target.
#
# Self-contained (same pattern as kist-ext-sensor-io): dependencies are
# cloned (pinned) and built INSIDE the image, models are downloaded, the repo
# source is baked in via COPY, and the binaries are built at image-build
# time — `docker run` gives ready binaries; no host-side vendoring, no bind
# mount, no in-container setup. The ONLY host-side piece is the XRoboToolkit
# PC service (the VR daemon: USB device + host service, see README); the
# container reaches it over --network host.
#
# For iterative dev you can still shadow the baked source with a bind mount
# (see run.sh comments).
FROM nvcr.io/nvidia/tensorrt:24.12-py3

ENV DEBIAN_FRONTEND=noninteractive

# ─── build tools + system deps ────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    ca-certificates \
    wget \
    libyaml-cpp-dev \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# ─── CycloneDDS idlc toolchain (VLA DDS types codegen) ────────────────────────
# 0.10.2 pinned to match unitree_sdk2's bundled libddscxx, so generated type
# headers stay ABI/format-compatible with the SDK's runtime libs. Installed
# to /opt as a BUILD tool only — deliberately no LD_LIBRARY_PATH (idlc finds
# its own libs via RUNPATH); exposing its libddsc at runtime would shadow the
# SDK's vendored .so. Same convention as kist-ext-sensor-io.
ARG CYCLONEDDS_VERSION=0.10.2
RUN git clone --depth 1 --branch "${CYCLONEDDS_VERSION}" \
        https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds && \
    cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DBUILD_IDLC=ON -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF -DENABLE_SECURITY=OFF && \
    cmake --build /tmp/cyclonedds/build --target install -j "$(nproc)" && \
    git clone --depth 1 --branch "${CYCLONEDDS_VERSION}" \
        https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git /tmp/cyclonedds-cxx && \
    cmake -S /tmp/cyclonedds-cxx -B /tmp/cyclonedds-cxx/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DCMAKE_PREFIX_PATH=/opt/cyclonedds -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF && \
    cmake --build /tmp/cyclonedds-cxx/build --target install -j "$(nproc)" && \
    rm -rf /tmp/cyclonedds /tmp/cyclonedds-cxx
ENV PATH=/opt/cyclonedds/bin:${PATH}
ENV CYCLONEDDS_HOME=/opt/cyclonedds

WORKDIR /workspace/kist-gearsonic-inference

# ─── vendored dep: PXREA VR client SDK (cloned + built + staged) ──────────────
# XRoboToolkit-PC-Service pinned; only the client library + headers are kept
# under thirdparty/pxrea (mirroring the manual README steps). The VR daemon
# itself runs on the HOST — this is just the client .so gearsonic links.
ARG XROBO_COMMIT=85bac4dbc1fd5cef42c74a160d9c30aa3491f122
RUN git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git /tmp/xrobo && \
    git -C /tmp/xrobo checkout "${XROBO_COMMIT}" && \
    bash /tmp/xrobo/RoboticsService/PXREARobotSDK/build.sh && \
    mkdir -p thirdparty/pxrea/include thirdparty/pxrea/lib && \
    cp /tmp/xrobo/RoboticsService/PXREARobotSDK/PXREARobotSDK.h thirdparty/pxrea/include/ && \
    cp -r /tmp/xrobo/RoboticsService/PXREARobotSDK/nlohmann thirdparty/pxrea/include/nlohmann && \
    cp /tmp/xrobo/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so thirdparty/pxrea/lib/ && \
    rm -rf /tmp/xrobo

# ─── vendored dep: unitree_sdk2 (cloned + pinned) ─────────────────────────────
# Same pin as kist-ext-sensor-io — its bundled libddscxx 0.10.2 matches the
# idlc built above, so generated DDS types stay ABI-compatible.
ARG UNITREE_SDK2_COMMIT=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2 && \
    git -C thirdparty/unitree_sdk2 checkout "${UNITREE_SDK2_COMMIT}"

# ─── VR daemon: XRoboToolkit PC service (in-image, amd64) ─────────────────────
# gearsonic needs the VR daemon (port 60061; it carries teleop tracking AND the
# e-stop). Bake it into the image so `source env.sh && run_vr_daemon` runs in
# the container — no host-side dpkg (same as Dockerfile.aarch64). The amd64 .deb
# is built for Ubuntu 24.04, matching this base, and bundles its own Qt6/ICU/
# openssl under /opt/apps/roboticsservice/lib (env.sh scopes LD_LIBRARY_PATH
# there) — so only glib + krb5 come from apt, no Qt/ICU wrangling. Its postinst
# is desktop-icon registration (xdg-icon-resource), pure GUI packaging cruft
# that exits non-zero and fails the build, so install via dpkg --unpack, stub
# the postinst, then --configure. The daemon runs headless and reaches the
# headset over WiFi (no USB passthrough / display).
ARG XROBO_DEB=XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
RUN apt-get update && apt-get install -y --no-install-recommends \
        libglib2.0-0t64 libgssapi-krb5-2 && \
    wget -q -O /tmp/xrobo.deb \
        "https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/${XROBO_DEB}" && \
    dpkg --unpack /tmp/xrobo.deb && \
    (echo '#!/bin/sh' > /var/lib/dpkg/info/roboticsservice.postinst; \
     chmod +x /var/lib/dpkg/info/roboticsservice.postinst) && \
    dpkg --configure roboticsservice && \
    rm -f /tmp/xrobo.deb && rm -rf /var/lib/apt/lists/*

# ─── models: GEAR-SONIC ONNX ──────────────────────────────────────────────────
# TensorRT engines (.trt) are generated next to them on first run.
RUN wget -q -P models \
    https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/model_encoder.onnx \
    https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/model_decoder.onnx \
    https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/planner_sonic.onnx

# ─── source + build ───────────────────────────────────────────────────────────
# Bake the repo in (build context = repo root, minus build*/, .git and the
# in-image vendored dirs via .dockerignore). The dep layers above stay
# cached; only this re-runs on a source change.
COPY . .
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j"$(nproc)"

CMD ["/bin/bash"]
