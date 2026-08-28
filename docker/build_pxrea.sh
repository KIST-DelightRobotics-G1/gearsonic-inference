#!/bin/bash
# Build the PXREA VR client SDK (libPXREARobotSDK.so) for aarch64 and stage
# it under thirdparty/pxrea/ for gearsonic to link. Used by
# docker/Dockerfile.aarch64 (the x86 Dockerfile uses upstream's build.sh
# directly — its vendored grpc bundle is complete there).
#
#   $1 = XRoboToolkit checkout dir (…/RoboticsService/PXREARobotSDK)
#   $2 = repo root (where thirdparty/pxrea/ is created)
#
# Why not upstream's build.sh on aarch64: XRoboToolkit ships an EMPTY aarch64
# grpc bundle (Redistributable/linux_aarch64/grpc/{include,lib} are just
# .gitkeep), and its pre-generated aarch64 stubs need protobuf 5.x
# (runtime_version.h) which apt doesn't provide. So we REGENERATE the stubs
# from the .proto with apt's protoc — the generated code then matches
# whatever protobuf apt ships — and compile the .so against apt grpc++.
set -euo pipefail

SDK_DIR="$1"      # …/PXREARobotSDK
REPO_ROOT="$2"
PROTO_DIR="${SDK_DIR}/../PXREAService"
GEN_DIR="$(mktemp -d)"

echo "[build_pxrea] aarch64 — regenerating gRPC stubs against apt grpc/protobuf"

# Well-known types (google/protobuf/empty.proto) ship with apt protobuf under
# /usr/include; -I both so the import resolves.
protoc -I"${PROTO_DIR}" -I/usr/include \
    --cpp_out="${GEN_DIR}" \
    --grpc_out="${GEN_DIR}" \
    --plugin=protoc-gen-grpc="$(which grpc_cpp_plugin)" \
    "${PROTO_DIR}/PXREAService.proto"

# SDK impl + freshly generated stubs -> libPXREARobotSDK.so, linked to apt
# grpc++ (pulls protobuf/absl transitively via pkg-config). -DLINUX_aarch64
# selects the Linux code paths in PXREARobotSDK.cpp (its <thread> include and
# the OutputDebug/thread members live behind
# `#if defined(LINUX_x86) || defined(LINUX_aarch64)`).
g++ -std=c++17 -fPIC -shared -O2 -DLINUX_aarch64 \
    -I"${SDK_DIR}" -I"${GEN_DIR}" \
    "${SDK_DIR}/PXREARobotSDK.cpp" \
    "${GEN_DIR}/PXREAService.pb.cc" \
    "${GEN_DIR}/PXREAService.grpc.pb.cc" \
    $(pkg-config --cflags --libs grpc++ protobuf) \
    -o "${GEN_DIR}/libPXREARobotSDK.so"

mkdir -p "${REPO_ROOT}/thirdparty/pxrea/include" \
         "${REPO_ROOT}/thirdparty/pxrea/lib"
cp "${SDK_DIR}/PXREARobotSDK.h"      "${REPO_ROOT}/thirdparty/pxrea/include/"
cp -r "${SDK_DIR}/nlohmann"          "${REPO_ROOT}/thirdparty/pxrea/include/nlohmann"
cp "${GEN_DIR}/libPXREARobotSDK.so"  "${REPO_ROOT}/thirdparty/pxrea/lib/"
echo "[build_pxrea] aarch64 libPXREARobotSDK.so built and staged"
