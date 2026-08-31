#!/bin/bash
# Build the container image, picking the Dockerfile for this machine's arch:
#   x86_64  -> docker/Dockerfile          (workstation)
#   aarch64 -> docker/Dockerfile.aarch64  (Jetson / onboard Orin)
# Same image tag either way, so run.sh is arch-agnostic.

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BUILD_NET=""
if [ "$(uname -m)" = "aarch64" ]; then
    DOCKERFILE="${REPO_DIR}/docker/Dockerfile.aarch64"
    # Jetson runs docker with "iptables": false (its kernel lacks the raw
    # table), which also disables bridge NAT — so bridge-networked RUN steps
    # have no route out. Build with host networking (same as how we run the
    # container) so apt/wget/git reach the internet.
    BUILD_NET="--network host"
else
    DOCKERFILE="${REPO_DIR}/docker/Dockerfile"
fi
echo "Building with ${DOCKERFILE}"

docker build ${BUILD_NET} \
    -t kist-gearsonic-inference \
    -f "${DOCKERFILE}" \
    "${REPO_DIR}"
