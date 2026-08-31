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

## 3. Power mode & clocks

Run at max power mode and lock the clocks to their peak (otherwise the
governor throttles the GPU/CPU and inference falls behind real time):

```bash
sudo nvpmodel -m 0      # MAXN — highest power mode
sudo jetson_clocks      # pin CPU/GPU/EMC clocks to max
```

`nvpmodel -m 0` persists across reboots; `jetson_clocks` does not — re-run it
after each boot (or install it as a boot service).

Once this system setup is done, build and run as on any host
([README](../README.md) Installation).
