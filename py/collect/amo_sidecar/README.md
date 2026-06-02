# amo_sidecar

Python-side glue for running the AMO whole-body policy
(`third_party/AMO`) as a sidecar to the C++ collect controller.

## Files

- `humanoid_env_headless.py` — `HumanoidEnvHeadless`, the policy core extracted
  from `third_party/AMO/play_amo.py` (no MuJoCo, no viewer). Accepts
  `(q23, dq23, quat, ang_vel, AmoCommands)`, returns 15 absolute PD targets
  for legs + waist (already including `default_dof_pos` offset).
- `wire_protocol.py` — fixed-layout little-endian wire format (state + action frames).
  C++ side must keep `cpp/g1/amo_bridge.hpp` byte-compatible with this.
- `amo_sidecar.py` — ZMQ entry point. SUB on state endpoint, PUB on action
  endpoint, drives `HumanoidEnvHeadless.step` at 50 Hz.

## Running the sidecar

```bash
cd py/collect/amo_sidecar
python amo_sidecar.py \
  --state-endpoint  tcp://127.0.0.1:5555 \
  --action-endpoint tcp://127.0.0.1:5556 \
  --device cpu
```

Defaults resolve model paths to `../../../third_party/AMO/`. Sidecar idles until the
first state frame arrives; SIGINT shuts it down cleanly.

## Setup

Use AMO's environment so versions match:

```bash
conda create -n amo python=3.8
conda activate amo
pip install -r ../../../third_party/AMO/requirements.txt
```

## Status

- [x] step 1: headless env extracted from play_amo
- [x] step 2: ZMQ wrapper (`amo_sidecar.py`) + wire format (`wire_protocol.py`)
- [ ] step 3: C++ `AmoBridge`
- [ ] step 4: collect.cpp integration in simulation
- [ ] step 5+: hardware
