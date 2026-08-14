# VLA mode (external latent tokens over DDS)

[kist-vla-inference](https://github.com/Safety-Node/kist-vla-inference) can
drive the whole body directly: it publishes 64-dim SONIC motion tokens +
Dex3 hand targets as `kist_msgs::LatentActionStep` on `rt/kist/latent_action`
(50 Hz), replacing the planner+encoder path.

## Contract

`idl/kist_msgs.idl` — shared with kist-vla-inference; built into
C++ types at build time by CycloneDDS `idlc` (0.10.2, matching the SDK's
bundled ddscxx — a required dependency, see the README's Install CycloneDDS;
the Docker image includes it). The Python side mirrors the types in
`kist_vla/io/dds.py`; keep the two in sync.

## Behavior

First-come arbitration (`include/control/control_arbiter.hpp`) between VLA
and teleop, with every exit routed through the origin (planner idle) —
never a direct handover:

- From the origin, a token fresher than 200 ms claims for VLA and switches
  `WholeBodyController` into external-token mode (planner path bypassed).
  Its hand targets replace the VR trigger mapping, clamped to the SDK
  joint limits. While VLA holds, the teleop B gesture is ignored.
- If teleop holds the robot (calibrated), VLA tokens are ignored until the
  operator disengages (B toggle) — then the origin re-arbitrates, so a
  still-live stream takes over from there.
- A stream stale for 500 ms is LOST and triggers the recovery sequence:
  blend (1 s) to a verified safe standing token, hold balancing there
  (hands keeping their last targets), reseed the planner from the measured
  standing pose, then return to the origin — locomotion disarmed, hands on
  their fallbacks. A resumed stream re-claims from the origin; no restart
  is needed.
- The VR e-stop (A+B+X+Y held 1s) outranks everything, always:
  damping, latched.

The safe standing token (`include/vla/vla_initial_pose.hpp`) is specific to
the public GEAR-SONIC checkpoint and mirrors kist-vla-inference's
`DEFAULT_INITIAL_MOTION_TOKEN` — re-derive both if the SONIC checkpoint
ever changes.

## Running

```bash
# gearsonic side (this repo) — same as teleop, VR still required as e-stop:
./build/kist-gearsonic-inference

# VLA side (kist-vla-inference), with a UNITREE_G1_SONIC checkpoint:
python scripts/run_vla.py --policy.model-path <ckpt> --io.action-transport dds

# Or without any model — stream the safe standing token (link check):
python scripts/publish_test_tokens.py --domain 0 --duration 15
```

## Interop probe

`./build/vla_receiver_probe [domain_id] [iface]` prints receive rate and the
newest token — pair it with a kist-vla-inference publisher to verify the
link without the robot.

`./build/motion_token_probe [domain_id] [iface]` subscribes the outgoing
decoder-token record stream (`rt/kist/motion_token`, for the data
collector) — run it next to a live gearsonic to verify rate, seq
continuity, and the mode tags.
