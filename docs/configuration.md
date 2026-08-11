# Configuration

All binaries read `config/config.yaml` (optional argv[1] overrides the path).
Edit it per deployment (on Docker, edit it inside the container — the image
bakes it in).

## `unitree`

| Key | Default | Meaning |
|---|---|---|
| `domain_id` | `0` | DDS domain — must match the robot (and the VLA publisher) |
| `network_interface` | `eno2` | NIC connected to the robot (e.g. `eno1`, `enp0s31f6`). Use `lo` for simulation |

## `planner`

| Key | Default | Meaning |
|---|---|---|
| `model_path` | `models/planner_sonic.onnx` | planner ONNX; the wrapper caches the built engine next to it as `.trt` |
| `precision` | `fp16` | TensorRT precision: `fp16` or `fp32` |
| `default_height` | `0.788740` | standing height for the initial planner context (gear_sonic default) |
| `initial_random_seed` | `1234` | random seed fed to the planner model |

## `control`

| Key | Default | Meaning |
|---|---|---|
| `encoder_path` | `models/model_encoder.onnx` | encoder: motion observation [1762] → token [64] |
| `decoder_path` | `models/model_decoder.onnx` | decoder (policy): observation [994] → joint action [29] |

## VLA-mode constants (compile-time)

The external-token behavior is tuned by constants, not YAML — they are
robot-safety parameters and change rarely:

| Constant | Where | Default | Meaning |
|---|---|---|---|
| `kVlaTokenFreshMs` | `include/vla/vla_latent_action.hpp` | `200` | a token this fresh claims the robot for VLA |
| `kVlaTokenHoldMs` | 〃 | `500` | staler than this in VLA mode = stream LOST |
| `kVlaLossBlendTicks` | 〃 | `50` (1 s) | blend length from the last token to the standing hold |
| `kVlaSafeStandingToken` | `include/vla/vla_initial_pose.hpp` | — | checkpoint-specific safe standing pose (see the header's warning) |
