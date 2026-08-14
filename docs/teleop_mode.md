# Teleop mode

Manual operation: the operator drives locomotion with the PICO controller
joysticks (planner path) and, after calibration, the arms and hands follow
the controllers (teleop path).

## 1. XRoboToolkit (PICO VR daemon)

With Docker, run this on the host, outside the container.

```bash
source env.sh
run_vr_daemon    # stop_vr_daemon to stop
```

Connect the headset from its XRoboToolkit app.

## 2. Control

**THE ROBOT MOVES on launch**: 3s ramp to standing, then policy control.

```bash
./build/kist-gearsonic-inference
```

## Controller

| Input | Action |
|---|---|
| Left stick | Move (magnitude = speed) |
| Right stick | Rotate facing |
| A | Return to IDLE |
| Y | Mode up (IDLE / Slow Walk / Walk) |
| Y held 1s | Mode up into hard actions (e.g. Run) — one hold per step |
| X | Mode down |
| Undefined | Height up / down (crouch modes) |
| B held 1s | Teleop on / off (engage in the reference pose: forearms 90° forward, palms inward) |
| Left / right grip (analog) | Left / right Dex3-1 thumb close (0 = open, 1 = pressed against fingers) |
| Left / right trigger (analog) | Left / right Dex3-1 index+middle close (0 = open, 1 = cage / fist) |
| A + B + X + Y held 1s | Emergency stop |

## Ownership vs VLA

Teleop and the VLA token stream share the robot first-come: whoever claims
first keeps it until it lets go, and every exit lands back at the origin
(planner idle). Calibrating teleop (B held 1s) claims the robot — VLA
tokens are ignored while you hold it. Toggling teleop off returns the robot
to the origin; if the VLA stream is still publishing, it takes over from
there. While VLA holds the robot, B is ignored — to take over, stop the
VLA publisher, let the robot recover to standing, then calibrate. See
[vla_mode.md](vla_mode.md) for the VLA side of the arbitration.
