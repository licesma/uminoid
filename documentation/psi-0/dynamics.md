# Psi-0 joint dynamics

This document describes how the Psi-0 stack drives the G1's joints at the
lowest level — specifically, how it populates the three fields of the Unitree
`MotorCmd` that actually shape closed-loop behavior:

- `kp` — proportional gain (stiffness)
- `kd` — derivative gain (damping)
- `tau` — feed-forward torque

The motor firmware on each joint runs a fixed PD + feed-forward law

```
tau_applied = kp * (q_target - q) + kd * (dq_target - dq) + tau_ff
```

at high rate. Psi-0's job is to choose `kp`, `kd`, `q_target`, and `tau_ff`
each control tick (`dq_target` is always 0 in Psi-0). Everything below is
about *how* it chooses them.

These values are shared across teleoperation, replay, and policy evaluation:
all three entry points instantiate the same `RobotTaskmaster` from
`third_party/Psi0/real/teleop/master_whole_body.py`, which in turn drives a
single `G1_29_BodyController` for the body and `Dex3_1_Controller` for the
hands. There is no separate "deploy" configuration of the low-level dynamics.

The values split into two groups by how they are produced:

1. **`kp` / `kd`** — fixed per-joint constants, hardcoded at controller
   initialization and never modified at runtime.
2. **`tau_ff`** — recomputed every control tick from a rigid-body model
   (Pinocchio's RNEA) of the desired pose. Configuration-dependent.

## Kp and Kd

`kp` and `kd` are fixed per joint and grouped by subsystem.

### Body (29 joints: legs, waist, arms + wrists)

Defined in `third_party/Psi0/real/teleop/robot_control/robot_body.py:191-212`
inside `G1_29_BodyController._setup_motor_params`. Written once into
`motor_cmd[id].kp` / `motor_cmd[id].kd` at line 219-220 and never updated.

| Subsystem | Joints | `kp` | `kd` |
|---|---|---|---|
| Hip pitch / roll / yaw | 3 per leg | 150 | 2 |
| Knee | 1 per leg | 300 | 4 |
| Ankle pitch | 1 per leg | 80 | 2 |
| Ankle roll | 1 per leg | 20 | 1 |
| Waist yaw / roll / pitch | 3 | 400 | 15 |
| Shoulder pitch / roll / yaw, elbow | 4 per arm | 50 | 7.5 |
| Wrist roll / pitch / yaw | 3 per arm | 30 | 6 |

The legs and waist values match those used to train the AMO locomotion
policy (`third_party/AMO/play_amo.py`), so changing them invalidates the
policy's priors. The arm values are a low-stiffness baseline chosen so the
exo operator does not fight stiff motors during teleop.

### Hands (14 joints: Dex3-1 fingers)

Defined in `third_party/Psi0/real/teleop/robot_control/robot_hand_unitree.py:58-59`
inside `Dex3_1_Controller.__init__`. Identical for every finger joint:

| `kp` | `kd` |
|---|---|
| 1.5 | 0.2 |

These are written once into `motor_cmd[id].kp` / `motor_cmd[id].kd` at
init (lines 69-70, 80-81) and never updated. The hands run on pure PD —
`motor_cmd[id].tau` is left at `0.0` and the runtime command loop
(lines 193-195) only writes `q`.
