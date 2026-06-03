# Bug: left arm state recorded as a hardcoded constant

## Symptom

In every `meta/stats_psi0.json` produced by the exo → LeRobot converter, the left-arm dims (14–20) of both `states` and `action` have **exact zero spread**:

```
dim 14: min=max=0.0      (shoulder pitch)
dim 15: min=max=0.0      (shoulder roll)
dim 16: min=max=0.0      (shoulder yaw)
dim 17: min=max=0.8      (elbow — park pose)
dim 18: min=max=0.0      (wrist roll)
dim 19: min=max=0.0      (wrist pitch)
dim 20: min=max=0.0      (wrist yaw)
```

`min == max` bit-exactly across every frame of every episode = the values are not coming from sensors. They are being written as a constant `[0, 0, 0, 0.8, 0, 0, 0]` park-pose vector by the collector.

## Why this is wrong

We have been running tasks that only command the right arm + right hand. The left arm is physically attached to the robot and is being:
- held in place by its low-level PD controller (tracking error ≠ 0)
- perturbed by dynamics from the rest of the robot moving
- subject to encoder noise

The true left-arm state is **not constant** — it drifts on the order of `1e-3`–`1e-2` rad around the park pose. Recording it as a literal constant throws away that information.

## Consequences

- **Training:** the left-arm state input has zero spread → after normalization it's identically `-1` for every frame on those dims. The model gets no proprioceptive signal for the left arm. Action targets on those dims are also exactly the constant, so the loss is trivially zero and the model learns "always output the constant" — fine in distribution, brittle out of it.
- **Deployment:** if the deployed left arm is anywhere other than `[0, 0, 0, 0.8, 0, 0, 0]` (boot pose drift, dynamics perturbation, hand-positioning by an operator), the model has no way to react. It will keep commanding the recorded constant regardless of what the arm is actually doing.
- **Future bimanual tasks:** any task that uses both arms requires this to be fixed before data collection.

## Fix

Record the **real** left-arm joint state from the low-level encoders, same way the right arm is recorded. The C++ collector should treat both arms symmetrically — read all 14 arm DoFs from the SDK, write them all to the CSV.

For the action column: if the left arm isn't being teleoperated, `action = state` (commanded = measured) is a reasonable label. Better is to log the actual desired_qpos the controller is sending — which for a held arm is a constant target, but state will drift around it, and the model learns the holding behavior.

## Related

- `psi0_migration_plan.md` — once this is fixed, Psi-0 Bugfix A (relaxed normalization tolerance) becomes unnecessary: the spread on those dims will be small but non-zero and well above the `== 0` threshold.
