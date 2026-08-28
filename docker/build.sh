#!/bin/bash
# Build the container image, picking the Dockerfile for this machine's arch:
#   x86_64  -> docker/Dockerfile          (workstation)
#   aarch64 -> docker/Dockerfile.aarch64  (Jetson / onboard Orin)
# Same image tag either way, so run.sh is arch-agnostic.

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "$(uname -m)" = "aarch64" ]; then
    DOCKERFILE="${REPO_DIR}/docker/Dockerfile.aarch64"
else
    DOCKERFILE="${REPO_DIR}/docker/Dockerfile"
fi
echo "Building with ${DOCKERFILE}"

docker build \
    -t kist-gearsonic-inference \
    -f "${DOCKERFILE}" \
    "${REPO_DIR}"
