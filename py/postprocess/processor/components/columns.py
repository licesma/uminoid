"""Validators for the merged data.csv produced by Postprocessor.

The merged frame is the contract between Postprocessor (producer) and the
LeRobot converter (consumer): schema, frame coverage, and per-collection
minimum length must all hold before conversion. Validating here — at the
moment data.csv is written — catches drift at its source rather than
letting it surface during conversion.
"""

from pathlib import Path

import pandas as pd

from helpers.error_check import ensure
from processor.lerobot.constants import (
    LEFT_ARM_COLS,
    LEFT_HAND_COLS,
    RIGHT_ARM_COLS,
    RIGHT_HAND_COLS,
)

META_COLS = ["collection_id", "frame", "host_timestamp"]
JOINT_COLS = [*LEFT_HAND_COLS, *RIGHT_HAND_COLS, *LEFT_ARM_COLS, *RIGHT_ARM_COLS]
EXPECTED_COLS = [*META_COLS, *JOINT_COLS]

# LeRobot encodes action[t] = state[t+1], so any collection with <2 frames
# cannot produce a valid (state, action) pair and must be rejected upstream.
MIN_FRAMES_PER_COLLECTION = 2


def validate_columns(df: pd.DataFrame) -> None:
    missing = [c for c in EXPECTED_COLS if c not in df.columns]
    ensure(not missing, f"data.csv missing expected columns: {missing}")


def validate_frames_exist(df: pd.DataFrame, frames_dir: Path) -> None:
    """Every PNG referenced in df["frame"] must exist on disk under frames_dir."""
    missing = [name for name in df["frame"].astype(str).unique()
               if not (frames_dir / name).is_file()]
    if missing:
        raise ValueError(
            f"{frames_dir} is missing {len(missing)} referenced PNG(s), e.g. {missing[0]}"
        )


def validate_collection_ids(df: pd.DataFrame) -> None:
    """Every collection_id must have at least MIN_FRAMES_PER_COLLECTION rows."""
    counts = df["collection_id"].value_counts()
    short = counts[counts < MIN_FRAMES_PER_COLLECTION]
    ensure(
        short.empty,
        f"collection_id(s) with <{MIN_FRAMES_PER_COLLECTION} frames: {short.to_dict()}",
    )
