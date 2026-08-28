#!/bin/bash
# Build the PXREA VR client SDK (libPXREARobotSDK.so) from an XRoboToolkit
# checkout, and stage it under thirdparty/pxrea/ for gearsonic to link.
#
#   $1 = XRoboToolkit checkout dir (…/RoboticsService/PXREARobotSDK)
#   $2 = repo root (where thirdparty/pxrea/ is created)
#
# Two paths by architecture:
#   x86_64  — upstream's own build.sh (its vendored grpc bundle is complete)
#   aarch64 — upstream ships an EMPTY aarch64 grpc bundle (include/ and lib/
#             are just .gitkeep), so its build.sh cannot work. We instead
#             REGENERATE the gRPC stubs from the .proto with the image's apt
#             protoc and compile the .so against apt's grpc++/protobuf. The
#             pre-generated aarch64 stubs need protobuf 5.x (runtime_version.h)
#             which apt doesn't ship — regenerating makes the code match
#             whatever protobuf apt provides, sidestepping the version gap.
set -euo pipefail

SDK_DIR="$1"      # …/PXREARobotSDK
REPO_ROOT="$2"
ARCH="$(uname -m)"

stage() {
    mkdir -p "${REPO_ROOT}/thirdparty/pxrea/include" \
             "${REPO_ROOT}/thirdparty/pxrea/lib"
    cp "${SDK_DIR}/PXREARobotSDK.h"      "${REPO_ROOT}/thirdparty/pxrea/include/"
    cp -r "${SDK_DIR}/nlohmann"          "${REPO_ROOT}/thirdparty/pxrea/include/nlohmann"
    cp "$1"                              "${REPO_ROOT}/thirdparty/pxrea/lib/libPXREARobotSDK.so"
}

if [ "${ARCH}" != "aarch64" ]; then
    echo "[build_pxrea] x86_64 — using upstream build.sh"
    bash "${SDK_DIR}/build.sh"
    stage "${SDK_DIR}/build/libPXREARobotSDK.so"
    exit 0
fi

echo "[build_pxrea] aarch64 — regenerating gRPC stubs against apt grpc/protobuf"
PROTO_DIR="${SDK_DIR}/../PXREAService"
GEN_DIR="$(mktemp -d)"

# Well-known types (google/protobuf/empty.proto) ship with apt protobuf under
# /usr/include; -I both so the import resolves.
protoc -I"${PROTO_DIR}" -I/usr/include \
    --cpp_out="${GEN_DIR}" \
    --grpc_out="${GEN_DIR}" \
    --plugin=protoc-gen-grpc="$(which grpc_cpp_plugin)" \
    "${PROTO_DIR}/PXREAService.proto"

# Build the .so: SDK impl + the freshly generated stubs, linked to apt grpc++.
# grpc++ pulls protobuf/absl transitively; pkg-config gives the right flags.
# -DLINUX_aarch64 selects the Linux code paths in PXREARobotSDK.cpp (the
# <thread> include and the OutputDebug/thread members live behind
# `#if defined(LINUX_x86) || defined(LINUX_aarch64)`; upstream's CMake sets
# the x86 one, we set the aarch64 one).
g++ -std=c++17 -fPIC -shared -O2 -DLINUX_aarch64 \
    -I"${SDK_DIR}" -I"${GEN_DIR}" \
    "${SDK_DIR}/PXREARobotSDK.cpp" \
    "${GEN_DIR}/PXREAService.pb.cc" \
    "${GEN_DIR}/PXREAService.grpc.pb.cc" \
    $(pkg-config --cflags --libs grpc++ protobuf) \
    -o "${GEN_DIR}/libPXREARobotSDK.so"

stage "${GEN_DIR}/libPXREARobotSDK.so"
echo "[build_pxrea] aarch64 libPXREARobotSDK.so built and staged"
