from pathlib import Path

import pandas as pd

from utils.joints import ARM_JOINTS

from helpers.error_check import ensure
from processor.components.mode import Mode
from processor.components.status import filter_completed

DISABLED_ARM_ELBOW_Q = 0.8
DISABLED_ARM_POSE = {j: (DISABLED_ARM_ELBOW_Q if j == "elbow" else 0.0) for j in ARM_JOINTS}

META_COLS = ["collection_id", "host_timestamp"]


def _read_arm_csv(path: Path) -> pd.DataFrame:
    """Load an arm CSV (already in robot-frame radians with named joint columns)."""
    df = pd.read_csv(path)
    if "timestamp" in df.columns:
        df = df.drop(columns=["timestamp"])
    return df


def _synthesize_disabled_arm(reference: pd.DataFrame, side: str) -> pd.DataFrame:
    """Build the missing `side` at the disabled-arm pose, mirroring the loaded
    side's joint columns (with the prefix swapped) so the two align."""
    other = "right" if side == "left" else "left"
    df = reference[META_COLS].copy()
    for col in reference.columns:
        if col in META_COLS:
            continue
        joint = col.removeprefix(f"{other}_").removesuffix("_joint")  # e.g. "elbow"
        df[f"{side}_{joint}_joint"] = DISABLED_ARM_POSE[joint]
    return df


def _load_side(episode_path: Path, side: str, mode: Mode, status_df: pd.DataFrame) -> pd.DataFrame | None:
    """Load one arm side per collection mode, filtered to completed collections.

    Returns None when the side has no recording at all (caller synthesizes it).
    Raises when a side is present but its mode-expected files are incomplete.
    """
    measured = episode_path / f"{side}_measured.csv"
    command = episode_path / f"{side}_command.csv"

    if mode is Mode.MOCAP:
        if not command.exists():
            return None
        df = _read_arm_csv(command)
    else:  # TELEOP: both files required per side; return measured.
        if not measured.exists() and not command.exists():
            return None
        ensure(
            measured.exists() and command.exists(),
            f"teleop expects both {measured.name} and {command.name} in {episode_path}",
        )
        df = _read_arm_csv(measured)

    if df.empty:
        return None
    return filter_completed(df, status_df)


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
