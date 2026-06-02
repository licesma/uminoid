from pathlib import Path

import pandas as pd

from utils.joints import ARM_JOINTS

from helpers.error_check import ensure
from processor.components.mode import Mode
from processor.components.status import filter_completed
from processor.lerobot.constants import ACTION_SUFFIX, STATE_SUFFIX

DISABLED_ARM_ELBOW_Q = 0.8
DISABLED_ARM_POSE = {j: (DISABLED_ARM_ELBOW_Q if j == "elbow" else 0.0) for j in ARM_JOINTS}

META_COLS = ["collection_id", "host_timestamp"]


def _read_arm_csv(path: Path, suffix: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    if "timestamp" in df.columns:
        df = df.drop(columns=["timestamp"])
    rename = {c: c + suffix for c in df.columns if c not in META_COLS}
    return df.rename(columns=rename)


def _synthesize_disabled_arm(reference: pd.DataFrame, side: str, suffix: str) -> pd.DataFrame:
    """Build the missing `side` at the disabled-arm pose, using `reference`'s
    timeline and emitting joint columns with the given suffix."""
    other = "right" if side == "left" else "left"
    df = reference[META_COLS].copy()
    for col in reference.columns:
        if col in META_COLS:
            continue
        # strip whichever suffix is on the reference, then swap the side prefix
        for s in (STATE_SUFFIX, ACTION_SUFFIX):
            if col.endswith(s):
                base = col[: -len(s)]
                joint = base.removeprefix(f"{other}_").removesuffix("_joint")
                df[f"{side}_{joint}_joint{suffix}"] = DISABLED_ARM_POSE[joint]
                break
    return df


def _load_side(
    episode_path: Path, side: str, mode: Mode, status_df: pd.DataFrame,
) -> tuple[pd.DataFrame | None, pd.DataFrame | None]:
    """Return (state_df, action_df) for one arm side, each carrying its own
    host_timestamp so the caller can align each independently against the
    camera timeline. MOCAP mirrors command into both. (None, None) when the
    side has no recording at all."""
    measured = episode_path / f"{side}_measured.csv"
    command = episode_path / f"{side}_command.csv"

    if mode is Mode.MOCAP:
        if not command.exists():
            return None, None
        state_df = _read_arm_csv(command, STATE_SUFFIX)
        action_df = _read_arm_csv(command, ACTION_SUFFIX)
    else:  # TELEOP: both files required per side.
        if not measured.exists() and not command.exists():
            return None, None
        ensure(
            measured.exists() and command.exists(),
            f"teleop expects both {measured.name} and {command.name} in {episode_path}",
        )
        state_df = _read_arm_csv(measured, STATE_SUFFIX)
        action_df = _read_arm_csv(command, ACTION_SUFFIX)

    if state_df.empty or action_df.empty:
        return None, None
    return filter_completed(state_df, status_df), filter_completed(action_df, status_df)


def load_arms(
    episode_path: Path, status_df: pd.DataFrame, mode: Mode,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """Returns (left_state, left_action, right_state, right_action)."""
    left_state, left_action = _load_side(episode_path, "left", mode, status_df)
    right_state, right_action = _load_side(episode_path, "right", mode, status_df)

    ensure(
        left_state is not None or right_state is not None,
        f"No arm recording (left or right) in {episode_path}",
    )

    if left_state is None:
        left_state = _synthesize_disabled_arm(right_state, "left", STATE_SUFFIX)
        left_action = _synthesize_disabled_arm(right_action, "left", ACTION_SUFFIX)
    if right_state is None:
        right_state = _synthesize_disabled_arm(left_state, "right", STATE_SUFFIX)
        right_action = _synthesize_disabled_arm(left_action, "right", ACTION_SUFFIX)

    return left_state, left_action, right_state, right_action
