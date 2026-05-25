"""
AMO sidecar: ZMQ wrapper around HumanoidEnvHeadless.

Subscribes to robot state frames published by the C++ collect process and
publishes 15 lower-body+waist PD targets at the AMO control rate (50 Hz).

Wire format: see wire_protocol.py. Endpoints are loopback-only.

Usage:
    python amo_sidecar.py \
        --policy ../third_party/AMO/amo_jit.pt \
        --adapter ../third_party/AMO/adapter_jit.pt \
        --norm-stats ../third_party/AMO/adapter_norm_stats.pt \
        --state-endpoint  tcp://127.0.0.1:5555 \
        --action-endpoint tcp://127.0.0.1:5556 \
        --device cpu

Behavior:
    - SUB socket uses CONFLATE=1: only the most-recent state frame is kept,
      so the policy always runs on fresh state and never builds backlog.
    - Until the first state frame arrives, the policy is not invoked.
    - Loop targets 50 Hz; if a tick overruns, the next tick fires immediately.
"""

import argparse
import csv
import signal
import struct
import sys
import time
from pathlib import Path

import numpy as np
import zmq

from humanoid_env_headless import HumanoidEnvHeadless, AmoCommands
import wire_protocol


CONTROL_HZ = 50.0
CONTROL_DT = 1.0 / CONTROL_HZ


def parse_args():
    p = argparse.ArgumentParser()
    here = Path(__file__).resolve().parent
    amo_dir = here.parent.parent.parent / "third_party" / "AMO"
    p.add_argument("--policy", default=str(amo_dir / "amo_jit.pt"))
    p.add_argument("--adapter", default=str(amo_dir / "adapter_jit.pt"))
    p.add_argument("--norm-stats", default=str(amo_dir / "adapter_norm_stats.pt"))
    p.add_argument("--state-endpoint", default="tcp://127.0.0.1:5555",
                   help="SUB endpoint where C++ publishes robot state")
    p.add_argument("--action-endpoint", default="tcp://127.0.0.1:5556",
                   help="PUB endpoint where we publish lower-body PD targets")
    p.add_argument("--device", default="cuda", choices=["cpu", "cuda"])
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--log-csv", default=None,
                   help="If set, append every (state, action) pair to this CSV "
                        "for offline analysis.")
    return p.parse_args()


def open_log(path):
    """Open a CSV writer for per-tick diagnostic dumps. Returns (file, writer)."""
    if path is None:
        return None, None
    f = open(path, "w", buffering=1)  # line-buffered so a crash still leaves data
    w = csv.writer(f)
    header = ["t_perf"]
    header += ["seq"]
    header += [f"q_{i}" for i in range(wire_protocol.N_JOINTS)]
    header += [f"dq_{i}" for i in range(wire_protocol.N_JOINTS)]
    header += ["qw", "qx", "qy", "qz"]
    header += ["wx", "wy", "wz"]
    header += ["vx", "yaw_target", "vy", "height",
               "torso_yaw", "torso_pitch", "torso_roll"]
    header += [f"qt_{i}" for i in range(wire_protocol.N_LOWER_TARGETS)]
    w.writerow(header)
    return f, w


class _Stop:
    def __init__(self):
        self.flag = False
        signal.signal(signal.SIGINT, self._handle)
        signal.signal(signal.SIGTERM, self._handle)

    def _handle(self, *_):
        self.flag = True


def main():
    args = parse_args()
    stop = _Stop()

    print(f"[sidecar] loading policy device={args.device}")
    env = HumanoidEnvHeadless(
        policy_path=args.policy,
        adapter_path=args.adapter,
        norm_stats_path=args.norm_stats,
        device=args.device,
    )

    ctx = zmq.Context.instance()

    state_sub = ctx.socket(zmq.SUB)
    state_sub.setsockopt(zmq.SUBSCRIBE, b"")
    state_sub.setsockopt(zmq.CONFLATE, 1)        # always read newest
    state_sub.setsockopt(zmq.RCVHWM, 1)
    state_sub.connect(args.state_endpoint)
    print(f"[sidecar] state SUB connected to {args.state_endpoint}")

    action_pub = ctx.socket(zmq.PUB)
    action_pub.setsockopt(zmq.SNDHWM, 1)
    action_pub.bind(args.action_endpoint)
    print(f"[sidecar] action PUB bound to {args.action_endpoint}")

    poller = zmq.Poller()
    poller.register(state_sub, zmq.POLLIN)

    log_file, log_writer = open_log(args.log_csv)
    if log_writer is not None:
        print(f"[sidecar] CSV log: {args.log_csv}")

    cmds = AmoCommands()
    last_state_seq = -1
    n_ticks = 0
    n_publish = 0
    next_tick = time.perf_counter()
    log_t = next_tick

    # latest_state persists across iterations: we accumulate the most recent
    # state across the ~300 polls that happen between two 50 Hz ticks. It gets
    # reset to None only AFTER we consume it on a tick.
    latest_state = None

    print("[sidecar] entering control loop @ 50 Hz")
    while not stop.flag:
        # Wait until either the next 20 ms tick or a fresh state frame.
        # Budget: at most CONTROL_DT remaining until next_tick.
        now = time.perf_counter()
        wait_ms = max(0.0, (next_tick - now) * 1000.0)
        # Block briefly so an idle loop doesn't burn CPU.
        events = dict(poller.poll(timeout=int(wait_ms)))

        if state_sub in events:
            # Drain (CONFLATE keeps only the newest; this is belt-and-suspenders).
            while True:
                try:
                    latest_state = state_sub.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break

        # Only fire a control tick on the cadence boundary.
        now = time.perf_counter()
        if now < next_tick:
            continue
        next_tick += CONTROL_DT
        # If we fell behind, snap forward to "now" so we don't spiral.
        if next_tick < now:
            next_tick = now + CONTROL_DT
        n_ticks += 1

        if latest_state is None:
            # No state yet -> no policy call, no action published. Idle.
            if args.verbose and (now - log_t) > 1.0:
                print("[sidecar] waiting for state...")
                log_t = now
            continue

        try:
            seq, ts_ns, q, dq, quat, ang_vel, cmd_arr = wire_protocol.unpack_state(latest_state)
        except Exception as e:
            print(f"[sidecar] bad state frame: {e}", file=sys.stderr)
            continue

        if seq == last_state_seq:
            # No new info; skip policy work.
            continue
        last_state_seq = seq

        cmds.vx = float(cmd_arr[0])
        cmds.yaw = float(cmd_arr[1])
        cmds.vy = float(cmd_arr[2])
        cmds.height = float(cmd_arr[3])
        cmds.torso_yaw = float(cmd_arr[4])
        cmds.torso_pitch = float(cmd_arr[5])
        cmds.torso_roll = float(cmd_arr[6])

        q_target = env.step(np.asarray(q), np.asarray(dq), np.asarray(quat),
                            np.asarray(ang_vel), cmds)

        out_ts = time.monotonic_ns()
        action_pub.send(wire_protocol.pack_action(seq, out_ts, q_target.tolist()))
        n_publish += 1

        if log_writer is not None:
            row = [time.perf_counter(), seq]
            row.extend(q)
            row.extend(dq)
            row.extend(quat)
            row.extend(ang_vel)
            row.extend(cmd_arr[:7])
            row.extend(q_target.tolist())
            log_writer.writerow(row)

        # Consumed: clear so the next batch of polls starts fresh. Without
        # this, a stale latest_state could persist across a sidecar pause and
        # we'd re-publish the same action with a different timestamp.
        latest_state = None

        if args.verbose and (now - log_t) > 1.0:
            print(f"[sidecar] ticks={n_ticks} pubs={n_publish} "
                  f"last_seq={seq} q_target[knee_L]={q_target[3]:+.3f}")
            log_t = now

    # Cleanup
    print(f"\n[sidecar] stopping (ticks={n_ticks}, pubs={n_publish})")
    if log_file is not None:
        log_file.close()
    state_sub.close(linger=0)
    action_pub.close(linger=0)
    ctx.term()
    return 0


if __name__ == "__main__":
    sys.exit(main())
