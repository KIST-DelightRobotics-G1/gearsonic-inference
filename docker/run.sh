#!/bin/bash
# Launch (or re-attach to) a persistent named container.
# Reuse across sessions so builds, caches, and background processes survive
# until you explicitly `docker rm kist-gearsonic-inference`.
#
# The image is self-contained (deps + models + source baked in and built);
# no source mount — `run.sh` drops you into a container with ready binaries
# under build/. Flags:
#   GPU access           x86: --gpus all ; Jetson: --runtime nvidia
#                        (the Orin rejects --gpus; it needs the NVIDIA
#                        Container Runtime instead)
#   --network host       unitree DDS + VLA DDS + the host-side VR daemon
#   --cap-add=SYS_NICE   RT thread priorities for the control loops
#
# Iterative dev: add  -v "$(pwd)":/workspace/kist-gearsonic-inference  to
# shadow the baked source with your working copy — then re-run the manual
# thirdparty/model setup from the README (the bind mount hides the baked
# copies) and rebuild inside.

set -e

CONTAINER=kist-gearsonic-inference

if [ "$(uname -m)" = "aarch64" ]; then
    GPU_FLAGS="--runtime nvidia"   # Jetson: NVIDIA Container Runtime
else
    GPU_FLAGS="--gpus all"         # x86: TensorRT via --gpus
fi

if [ "$(docker ps -q -f name=^${CONTAINER}$)" ]; then
    # Already running → attach a new shell to it
    docker exec -it "${CONTAINER}" /bin/bash
elif [ "$(docker ps -aq -f name=^${CONTAINER}$)" ]; then
    # Exists but stopped → start it back up
    docker start -ai "${CONTAINER}"
else
    # First run → create it
    docker run -it \
        --name "${CONTAINER}" \
        ${GPU_FLAGS} \
        --network host \
        --cap-add=SYS_NICE \
        -w /workspace/kist-gearsonic-inference \
        kist-gearsonic-inference
fi
