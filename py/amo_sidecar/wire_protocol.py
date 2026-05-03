"""
Wire format for the C++ <-> Python AMO sidecar bridge.

Localhost only, fixed-layout little-endian binary. Keep this file in sync with
the matching C++ struct in cpp/g1/amo_bridge.hpp.

State frame (C++ -> Python), 496 bytes:
  uint64   seq
  uint64   ts_ns                   monotonic ns at publish time
  float64  q[23]                   joint positions, AMO 23-DoF order
  float64  dq[23]                  joint velocities, AMO 23-DoF order
  float64  quat[4]                 body orientation (w, x, y, z)
  float64  ang_vel[3]              body angular velocity
  float64  cmds[7]                 vx, yaw_target, vy, height,
                                   torso_yaw, torso_pitch, torso_roll

Action frame (Python -> C++), 136 bytes:
  uint64   seq                     echoes state.seq used to produce this action
  uint64   ts_ns                   monotonic ns at publish time
  float64  q_target[15]            absolute PD targets, lower body + waist,
                                   AMO 15-DoF order (legs L+R, then waist)
"""

import struct

# Sizes in DoF / dim
N_JOINTS = 23
N_QUAT = 4
N_ANG_VEL = 3
N_CMDS = 7
N_LOWER_TARGETS = 15

_STATE_FMT = "<QQ" + f"{N_JOINTS}d" + f"{N_JOINTS}d" + f"{N_QUAT}d" + f"{N_ANG_VEL}d" + f"{N_CMDS}d"
_ACTION_FMT = "<QQ" + f"{N_LOWER_TARGETS}d"

STATE_SIZE = struct.calcsize(_STATE_FMT)      # 496
ACTION_SIZE = struct.calcsize(_ACTION_FMT)    # 136


def pack_state(seq, ts_ns, q, dq, quat, ang_vel, cmds):
    return struct.pack(_STATE_FMT, seq, ts_ns, *q, *dq, *quat, *ang_vel, *cmds)


def unpack_state(buf):
    if len(buf) != STATE_SIZE:
        raise ValueError(f"state frame size {len(buf)} != {STATE_SIZE}")
    fields = struct.unpack(_STATE_FMT, buf)
    seq, ts_ns = fields[0], fields[1]
    o = 2
    q = fields[o:o + N_JOINTS]; o += N_JOINTS
    dq = fields[o:o + N_JOINTS]; o += N_JOINTS
    quat = fields[o:o + N_QUAT]; o += N_QUAT
    ang_vel = fields[o:o + N_ANG_VEL]; o += N_ANG_VEL
    cmds = fields[o:o + N_CMDS]
    return seq, ts_ns, q, dq, quat, ang_vel, cmds


def pack_action(seq, ts_ns, q_target):
    return struct.pack(_ACTION_FMT, seq, ts_ns, *q_target)


def unpack_action(buf):
    if len(buf) != ACTION_SIZE:
        raise ValueError(f"action frame size {len(buf)} != {ACTION_SIZE}")
    fields = struct.unpack(_ACTION_FMT, buf)
    return fields[0], fields[1], fields[2:]
