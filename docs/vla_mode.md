# VLA mode (external latent tokens over DDS)

[kist-vla-inference](https://github.com/Safety-Node/kist-vla-inference) can
drive the whole body directly: it publishes 64-dim SONIC motion tokens +
Dex3 hand targets as `kist_msgs::LatentActionStep` on `rt/kist/latent_action`
(50 Hz), replacing the planner+encoder path.

## Contract

`idl/kist_latent_action.idl` — shared with kist-vla-inference; built into
C++ types at build time by CycloneDDS `idlc` (0.10.2, matching the SDK's
bundled ddscxx — a required dependency, see the README's Install CycloneDDS;
the Docker image includes it). The Python side mirrors the types in
`kist_vla/io/dds.py`; keep the two in sync.

## Behavior

First-come ownership (`include/control/robot_ownership.hpp`) arbitrates VLA
vs teleop — whichever claims the robot first keeps it:

- A token fresher than 200 ms claims for VLA and switches
  `WholeBodyController` into external-token mode (planner path bypassed).
  Its hand targets replace the VR trigger mapping, clamped to the SDK
  joint limits.
- If teleop calibrated first, VLA tokens are ignored until restart.
- VLA ownership is sticky — a stream stale for 500 ms counts as LOST and
  the controller blends (1 s) to a verified safe standing token and keeps
  balancing there, hands holding their last targets; neither a resumed
  stream nor the planner path (its playback timeline froze at the pre-VLA
  state) may retake the robot — restart to recover.
- The VR grip e-stop (A+B+X+Y held 1s) outranks everything, always:
  damping, latched.

The safe standing token (`include/vla/vla_initial_pose.hpp`) is specific to
the public GEAR-SONIC checkpoint and mirrors kist-vla-inference's
`DEFAULT_INITIAL_MOTION_TOKEN` — re-derive both if the SONIC checkpoint
ever changes.

## Running

```bash
# gearsonic side (this repo) — same as teleop, VR still required as e-stop:
./build/gearsonic_inference

# VLA side (kist-vla-inference), with a UNITREE_G1_SONIC checkpoint:
python scripts/run_vla.py --policy.model-path <ckpt> --io.action-transport dds

# Or without any model — stream the safe standing token (link check):
python scripts/publish_test_tokens.py --domain 0 --duration 15
```

## Interop probe

`./build/vla_receiver_probe [domain_id] [iface]` prints receive rate and the
newest token — pair it with a kist-vla-inference publisher to verify the
link without the robot.
