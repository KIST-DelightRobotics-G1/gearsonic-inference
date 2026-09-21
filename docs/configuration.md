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
| `encoder_path` | `models/model_encoder.onnx` | encoder: motion observation [1751] → token [64] |
| `decoder_path` | `models/model_decoder.onnx` | decoder (policy): observation [994] → joint action [29] |

## `hand`

Dex3-1 feedback guard between the trigger/VLA target and the published
command. All keys optional; the hand state stream absent or stale falls back
to the open-loop command unchanged.

| Key | Default | Meaning |
|---|---|---|
| `guard_enabled` | `true` | run the guard at all |
| `kp`, `kd` | `1.5`, `0.1` | PD gains while hand state is fresh |
| `tau_max` | `0.6` | stall torque bound; the target is kept within `tau_max / kp` rad of the measured position |
| `stall_err_th`, `stall_vel_th`, `stall_time_s` | `0.15`, `0.05`, `0.3` | stall latch: error above / speed below these for this long |
| `stall_offset`, `kp_hold` | `0.25`, `1.5` | while latched: target = measured + offset toward the goal, at this kp |
| `temp_max_c` | `0` | casing temperature that latches immediately; `0` = off |
| `state_stale_ms` | `100` | hand state older than this counts as absent |

## `motor_health`

Fault / temperature observer over the body and hand state. Log-only: it
writes `[MotorHealth]` lines and changes no command. All keys optional.

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | run the observer |
| `casing_warn_c`, `casing_critical_c` | `72`, `76` | casing temperature WARNING / EMERGENCY (reference cut-out 85) |
| `winding_warn_c`, `winding_critical_c` | `110`, `117` | winding temperature WARNING / EMERGENCY (measured cut-out 130: motor fault, then the whole robot loses torque) |
| `unresp_tau_min`, `unresp_tau_ratio`, `unresp_time_s` | `5.0`, `0.3`, `1.0` | NOT RESPONDING: commanded torque above `tau_min` Nm while `tau_est` stays below `tau_ratio` of it for this long |
| `frozen_time_s`, `frozen_cmd_move` | `1.0`, `0.02` | STATE FROZEN: measured q bit-identical for this long while the command moved more than this |
| `summary_period_s` | `0` | periodic hottest-motor / faults line; `0` = off (turn on for thermal tests) |

## VLA-mode constants (compile-time)

The external-token behavior is tuned by constants, not YAML — they are
robot-safety parameters and change rarely:

| Constant | Where | Default | Meaning |
|---|---|---|---|
| `kVlaTokenFreshMs` | `include/vla/vla_latent_action.hpp` | `200` | a token this fresh claims the robot for VLA |
| `kVlaTokenHoldMs` | 〃 | `500` | staler than this in VLA mode = stream LOST |
| `kVlaLossBlendTicks` | 〃 | `50` (1 s) | blend length from the last token to the standing hold |
| `kVlaSafeStandingToken` | `include/vla/vla_initial_pose.hpp` | — | checkpoint-specific safe standing pose (see the header's warning) |
