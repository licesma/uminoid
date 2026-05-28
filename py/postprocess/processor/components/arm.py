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


def _synthesize_disabled_arm(reference: pd.DataFrame, side: str) -> pd.DataFrame:
    """Build the missing `side` at the disabled-arm pose, mirroring the loaded
    side's joint columns."""
    other = "right" if side == "left" else "left"
    df = reference[META_COLS].copy()
    for col in reference.columns:
        if col in META_COLS:
            continue
        # col is like "right_elbow_joint_state"; pull off side prefix + suffix to get "elbow"
        for suffix in (STATE_SUFFIX, ACTION_SUFFIX):
            if col.endswith(suffix):
                base = col[: -len(suffix)]
                joint = base.removeprefix(f"{other}_").removesuffix("_joint")
                df[f"{side}_{joint}_joint{suffix}"] = DISABLED_ARM_POSE[joint]
                break
    return df


def _load_side(episode_path: Path, side: str, mode: Mode, status_df: pd.DataFrame) -> pd.DataFrame | None:
    """Load one arm side, returning a df with both _state-suffixed (measured)
    and _action-suffixed (command) joint columns, filtered to completed
    collections."""
    measured = episode_path / f"{side}_measured.csv"
    command = episode_path / f"{side}_command.csv"

    if mode is Mode.MOCAP:
        if not command.exists():
            return None
        cmd_df = _read_arm_csv(command, ACTION_SUFFIX)
        state_df = _read_arm_csv(command, STATE_SUFFIX)
        merged = pd.merge(cmd_df, state_df, on=META_COLS, how="inner")
    else:  # TELEOP: both files required per side.
        if not measured.exists() and not command.exists():
            return None
        ensure(
            measured.exists() and command.exists(),
            f"teleop expects both {measured.name} and {command.name} in {episode_path}",
        )
        state_df = _read_arm_csv(measured, STATE_SUFFIX).sort_values("host_timestamp")
        action_df = _read_arm_csv(command, ACTION_SUFFIX).sort_values("host_timestamp")
        # measured and command stream at independent rates; align action to
        # the state timeline by nearest host_timestamp within the collection.
        merged = pd.merge_asof(
            state_df, action_df,
            on="host_timestamp", by="collection_id", direction="nearest",
        )

    if merged.empty:
        return None
    return filter_completed(merged, status_df)


def load_arms(episode_path: Path, status_df: pd.DataFrame, mode: Mode) -> tuple[pd.DataFrame, pd.DataFrame]:
    left = _load_side(episode_path, "left", mode, status_df)
    right = _load_side(episode_path, "right", mode, status_df)

    ensure(
        left is not None or right is not None,
        f"No arm recording (left or right) in {episode_path}",
    )

    return (
        left if left is not None else _synthesize_disabled_arm(right, "left"),
        right if right is not None else _synthesize_disabled_arm(left, "right"),
    )
