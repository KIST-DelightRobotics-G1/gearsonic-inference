# Jetson (onboard Orin) setup

Running gearsonic on the robot's onboard Orin removes the PC↔robot link
entirely. This covers the one-time setup from a freshly flashed JetPack;
after it, use `docker/build.sh` / `docker/run.sh` as on any host.

## 1. JetPack 6

Flash the Orin to JetPack 6 (L4T R36 / Ubuntu 22.04):

- https://nvlabs.github.io/GR00T-WholeBodyControl/references/jetpack6.html

## 2. Docker daemon

Set `/etc/docker/daemon.json`, then `sudo systemctl restart docker`:

```json
{
  "runtimes": { "nvidia": { "path": "nvidia-container-runtime", "args": [] } },
  "default-runtime": "nvidia",
  "iptables": false
}
```

- `default-runtime: nvidia` — injects the host's CUDA/TensorRT/DLA libs at
  BUILD time too, not just at run; without it the link step fails with
  `undefined reference to nvdla::`.
- `iptables: false` — the Orin kernel lacks the iptables `raw` table; without
  this docker networking fails to initialize. `build.sh` and `run.sh` use
  `--network host`, which needs no bridge NAT.
- `nvidia-ctk runtime configure --runtime=docker` writes the `runtimes` entry.

## 3. VR daemon

Nothing to install on the host — the arm64 daemon is built for Ubuntu 22.04
and won't run on the Orin, so `Dockerfile.aarch64` bakes the headless daemon
INTO the image. Start it from inside the container:

```bash
source env.sh && run_vr_daemon      # daemon on :60061, in-container
```

## 4. Build & run

`build.sh` auto-selects `Dockerfile.aarch64` on aarch64; same image tag, so
`run.sh` is identical to x86.

```bash
./docker/build.sh
./docker/run.sh
```

Set `unitree.network_interface` to the Orin's robot-facing NIC (e.g.
`enP8p1s0`, check `ip -br addr`) — edit `config/config.yaml` inside the
running container.
