"""
Headless extraction of AMO's HumanoidEnv (third_party/AMO/play_amo.py).

Removes MuJoCo, mujoco_viewer, and glfw. Caller supplies the robot state
(q23, dq23, quat, ang_vel) and high-level commands; receives 15 absolute
PD targets for the lower body + waist (already including default_dof_pos).

Constants and execution order are copied verbatim from play_amo.py so that
parity_test.py can assert bit-equivalence with the reference implementation.
"""

from collections import deque
import numpy as np
import torch


# AMO 23-DoF order (from play_amo.HumanoidEnv.dof_names):
#   0..5    left leg   (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
#   6..11   right leg
#   12..14  waist      (yaw, roll, pitch)
#   15..18  left arm   (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow)
#   19..22  right arm
LOWER_BODY_DIM = 15
ARM_DIM = 8
NUM_DOFS = 23


def quat_to_euler(quat):
    """Identical to play_amo.quatToEuler. quat = (w, x, y, z)."""
    eulerVec = np.zeros(3)
    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    eulerVec[0] = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    if np.abs(sinp) >= 1:
        eulerVec[1] = np.copysign(np.pi / 2, sinp)
    else:
        eulerVec[1] = np.arcsin(sinp)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    eulerVec[2] = np.arctan2(siny_cosp, cosy_cosp)
    return eulerVec


class AmoCommands:
    """High-level commands for the policy. Mirrors viewer.commands[0..6].

    play_amo.py's `commands[7]` is the arm-toggle. We do not expose it because
    the C++ side owns arm targets (driven by the exo). The headless env only
    returns the 15 lower-body+waist PD targets.
    """

    __slots__ = ("vx", "yaw", "vy", "height", "torso_yaw", "torso_pitch", "torso_roll")

    def __init__(self):
        self.vx = 0.0          # commands[0]
        self.yaw = 0.0         # commands[1]  (target yaw)
        self.vy = 0.0          # commands[2]
        self.height = 0.0      # commands[3]  (offset from 0.75 m)
        self.torso_yaw = 0.0   # commands[4]
        self.torso_pitch = 0.0 # commands[5]
        self.torso_roll = 0.0  # commands[6]

    def to_array(self):
        return np.array([
            self.vx, self.yaw, self.vy, self.height,
            self.torso_yaw, self.torso_pitch, self.torso_roll, 0.0,
        ], dtype=np.float32)


class HumanoidEnvHeadless:
    """Pure-policy core extracted from play_amo.HumanoidEnv (g1 branch)."""

    def __init__(self, policy_path, adapter_path, norm_stats_path, device="cpu"):
        self.device = device

        # ---- Constants copied verbatim from play_amo.HumanoidEnv ------------
        self.num_actions = 15
        self.num_dofs = NUM_DOFS
        self.default_dof_pos = np.array([
            -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
            -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
             0.0, 0.0, 0.0,
             0.5, 0.0, 0.2, 0.3,
             0.5, 0.0, -0.2, 0.3,
        ], dtype=np.float32)

        self.action_scale = 0.25
        self.scales_ang_vel = 0.25
        self.scales_dof_vel = 0.05

        self.nj = NUM_DOFS
        self.n_priv = 3
        self.n_proprio = 3 + 2 + 2 + 23 * 3 + 2 + 15
        self.history_len = 10
        self.extra_history_len = 25
        self._n_demo_dof = 8

        # Control rate. play_amo.py uses sim_dt=0.002 and decimation=10 -> 50 Hz.
        self.control_dt = 0.02
        self.gait_freq = 1.3

        # ---- Persistent state ------------------------------------------------
        self.last_action = np.zeros(self.nj, dtype=np.float32)
        self.gait_cycle = np.array([0.25, 0.25], dtype=np.float64)
        self._in_place_stand_flag = True

        self.demo_obs_template = np.zeros((8 + 3 + 3 + 3,), dtype=np.float32)
        self.demo_obs_template[: self._n_demo_dof] = self.default_dof_pos[15:]
        self.demo_obs_template[self._n_demo_dof + 6: self._n_demo_dof + 9] = 0.75

        self.proprio_history_buf = deque(maxlen=self.history_len)
        self.extra_history_buf = deque(maxlen=self.extra_history_len)
        for _ in range(self.history_len):
            self.proprio_history_buf.append(np.zeros(self.n_proprio, dtype=np.float32))
        for _ in range(self.extra_history_len):
            self.extra_history_buf.append(np.zeros(self.n_proprio, dtype=np.float32))

        # ---- Models ----------------------------------------------------------
        self.policy_jit = torch.jit.load(policy_path, map_location=device)
        self.policy_jit.eval()
        for p in self.policy_jit.parameters():
            p.requires_grad = False

        self.adapter = torch.jit.load(adapter_path, map_location=device)
        self.adapter.eval()
        for p in self.adapter.parameters():
            p.requires_grad = False

        norm_stats = torch.load(norm_stats_path, map_location=device)
        self.input_mean = torch.tensor(norm_stats["input_mean"], device=device, dtype=torch.float32)
        self.input_std = torch.tensor(norm_stats["input_std"], device=device, dtype=torch.float32)
        self.output_mean = torch.tensor(norm_stats["output_mean"], device=device, dtype=torch.float32)
        self.output_std = torch.tensor(norm_stats["output_std"], device=device, dtype=torch.float32)

    def step(self, q23, dq23, quat, ang_vel, commands):
        """One control tick.

        Args:
          q23:     (23,) joint positions in AMO order (see top of file)
          dq23:    (23,) joint velocities, same order
          quat:    (4,)  body orientation as (w, x, y, z)
          ang_vel: (3,)  body angular velocity
          commands: AmoCommands

        Returns:
          (15,) absolute PD targets for the lower body + waist (legs + waist),
          already including default_dof_pos[:15].
        """
        q23 = np.asarray(q23, dtype=np.float32)
        dq23 = np.asarray(dq23, dtype=np.float32)
        quat = np.asarray(quat, dtype=np.float32)
        ang_vel = np.asarray(ang_vel, dtype=np.float32)
        cmds = commands.to_array()

        # ----- Observation construction (mirrors HumanoidEnv.get_observation)
        rpy = quat_to_euler(quat)

        target_yaw = cmds[1]
        dyaw = rpy[2] - target_yaw
        dyaw = np.remainder(dyaw + np.pi, 2 * np.pi) - np.pi
        if self._in_place_stand_flag:
            dyaw = 0.0

        gait_obs = np.sin(self.gait_cycle * 2 * np.pi)

        # Adapter input: [height, torso_yaw, torso_pitch, torso_roll, arm_q(8)]
        adapter_input = np.concatenate([np.zeros(4, dtype=np.float32), q23[15:]])
        adapter_input[0] = 0.75 + cmds[3]
        adapter_input[1] = cmds[4]
        adapter_input[2] = cmds[5]
        adapter_input[3] = cmds[6]
        adapter_input_t = torch.tensor(adapter_input, device=self.device, dtype=torch.float32).unsqueeze(0)
        adapter_input_t = (adapter_input_t - self.input_mean) / (self.input_std + 1e-8)
        adapter_output_t = self.adapter(adapter_input_t.view(1, -1))
        adapter_output_t = adapter_output_t * self.output_std + self.output_mean
        adapter_out = adapter_output_t.detach().cpu().numpy().squeeze()

        obs_prop = np.concatenate([
            ang_vel * self.scales_ang_vel,
            rpy[:2],
            np.array([np.sin(dyaw), np.cos(dyaw)], dtype=np.float32),
            (q23 - self.default_dof_pos),
            dq23 * self.scales_dof_vel,
            self.last_action,
            gait_obs,
            adapter_out,
        ]).astype(np.float32)

        obs_priv = np.zeros((self.n_priv,), dtype=np.float32)
        # NOTE: obs_hist is read BEFORE appending obs_prop, matching play_amo.
        obs_hist = np.array(self.proprio_history_buf, dtype=np.float32).flatten()

        obs_demo = self.demo_obs_template.copy()
        obs_demo[: self._n_demo_dof] = q23[15:]
        obs_demo[self._n_demo_dof + 0] = cmds[0]
        obs_demo[self._n_demo_dof + 1] = cmds[2]
        # _in_place_stand_flag is updated inside get_observation in play_amo.py.
        self._in_place_stand_flag = bool(np.abs(cmds[0]) < 0.1)
        obs_demo[self._n_demo_dof + 3] = cmds[4]
        obs_demo[self._n_demo_dof + 4] = cmds[5]
        obs_demo[self._n_demo_dof + 5] = cmds[6]
        obs_demo[self._n_demo_dof + 6: self._n_demo_dof + 9] = 0.75 + cmds[3]

        # Append AFTER obs_hist read; matches play_amo.py.
        self.proprio_history_buf.append(obs_prop)
        self.extra_history_buf.append(obs_prop)

        obs = np.concatenate((obs_prop, obs_demo, obs_priv, obs_hist)).astype(np.float32)

        # ----- Policy call (mirrors HumanoidEnv.run inner block) -----
        obs_tensor = torch.from_numpy(obs).float().unsqueeze(0).to(self.device)
        extra_hist = torch.tensor(
            np.array(self.extra_history_buf, dtype=np.float32).flatten().copy(),
            dtype=torch.float,
        ).view(1, -1).to(self.device)
        with torch.no_grad():
            raw_action = self.policy_jit(obs_tensor, extra_hist).cpu().numpy().squeeze()
        raw_action = np.clip(raw_action, -40.0, 40.0)
        self.last_action = np.concatenate([
            raw_action.copy(),
            (q23 - self.default_dof_pos)[15:] / self.action_scale,
        ]).astype(np.float32)
        scaled_actions = raw_action * self.action_scale

        # ----- Gait cycle update (mirrors run() post-policy block) -----
        self.gait_cycle = np.remainder(self.gait_cycle + self.control_dt * self.gait_freq, 1.0)
        if self._in_place_stand_flag and (
            (np.abs(self.gait_cycle[0] - 0.25) < 0.05)
            or (np.abs(self.gait_cycle[1] - 0.25) < 0.05)
        ):
            self.gait_cycle = np.array([0.25, 0.25])
        if (not self._in_place_stand_flag) and (
            (np.abs(self.gait_cycle[0] - 0.25) < 0.05)
            and (np.abs(self.gait_cycle[1] - 0.25) < 0.05)
        ):
            self.gait_cycle = np.array([0.25, 0.75])

        # play_amo: pd_target[:15] = scaled_actions + default_dof_pos[:15]
        pd_target_lower = scaled_actions + self.default_dof_pos[:15]
        return pd_target_lower.astype(np.float32)
