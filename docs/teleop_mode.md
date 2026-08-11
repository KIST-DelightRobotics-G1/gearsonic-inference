# Teleop mode

Manual operation: the operator drives locomotion with the PICO controller
joysticks (planner path) and, after calibration, the arms and hands follow
the controllers (teleop path).

## 1. XRoboToolkit (PICO VR daemon)

With Docker, run this on the host, outside the container.

```bash
source env.sh
run_vr_daemon
```

Connect the headset from its XRoboToolkit app.

## 2. Control

**THE ROBOT MOVES on launch**: 3s ramp to standing, then policy control.

```bash
./build/gearsonic_inference
```

## Controller

| Input | Action |
|---|---|
| Left stick | Move (magnitude = speed) |
| Right stick | Rotate facing |
| A | Return to IDLE |
| Y | Mode up (IDLE / Slow Walk / Walk) |
| Trigger + Y | Mode up (hard actions, e.g. Run) |
| X | Mode down |
| Trigger + B / A | Height up / down (crouch modes) |
| B held 1s | Teleop on / off (engage in the reference pose: forearms 90° forward, palms inward) |
| Left / right grip (analog) | Left / right Dex3-1 thumb close (0 = open, 1 = pressed against fingers) |
| Left / right trigger (analog) | Left / right Dex3-1 index+middle close (0 = open, 1 = cage / fist) |
| A + B + X + Y held 1s | Emergency stop |

## Ownership vs VLA

Teleop and the VLA token stream share the robot under first-come ownership:
whoever claims first keeps it. Calibrating teleop (B held 1s) claims the
robot — VLA tokens arriving afterwards are ignored until restart. See
[vla_mode.md](vla_mode.md) for the VLA side of the arbitration.
