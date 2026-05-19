#!/usr/bin/env python3
"""Postprocess a recorded session into a data.csv ready for the
Humanoid-Exo-Learning Dex3 pipeline (third_party/scripts/data/raw_he_to_psi0.py).

For each camera frame (the master timeline), finds the nearest sample by
host_timestamp — within the same collection_id — from the arm CSVs and
``dex3_hand.csv``.  Output columns and ordering match the 28-D G1 state vector
used in raw_he_to_psi0.py::action_to_joint_cfg_g1:

    [collection_id, frame, host_timestamp,
     LEFT_HAND_JOINTS_G1  (7), RIGHT_HAND_JOINTS_G1 (7),
     LEFT_ARM_JOINTS_G1   (7), RIGHT_ARM_JOINTS_G1  (7)]

Only the measured state (``*_actual_*``) is kept from ``dex3_hand.csv``; cmd,
force, and tactile pressure streams are dropped.

Only ``collection_id``s whose ``collection_status.csv`` entry is
``completed`` are kept; any other status (e.g. ``cancelled``) is dropped
before the merge.

Each arm side is auto-detected per-file:
- ``{side}_measured.csv`` — joint_0..joint_N already in robot-frame radians.
- ``{side}_arm.csv``      — raw dynamixel encoder bits; converted via
  ``to_robot_angle`` using the exo/G1 joint bounds.
``_measured`` takes precedence if both are present.  If exactly one side has
no arm recording, the missing side is synthesized at the disabled-arm initial
pose used by ``cpp/g1/g1Controller.cpp`` (all zeros except elbow=
``DISABLED_ARM_ELBOW_Q``), sharing host_timestamps with the side that does
have a recording.  If both sides are missing the script errors out.

Usage:
    python postprocess_data.py april_19_16:31
"""

import argparse
from pathlib import Path

import pandas as pd
import yaml

from joints import ARM_JOINTS
from utils.convert_to_radian import to_robot_angle

REPO_ROOT = Path(__file__).parent.parent
EXO_BOUNDS_PATH = REPO_ROOT / "cpp/upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
G1_BOUNDS_PATH  = REPO_ROOT / "cpp/g1/model/upperBodyJointBounds.yaml"

META_COLS = ["collection_id", "frame", "host_timestamp"]

# Joint names and order taken verbatim from
# third_party/Humanoid-Exo-Learning/scripts/data/raw_he_to_psi0.py.
# The asymmetric index/middle order on the right side is intentional — it
# matches the Unitree Dex3 firmware message layout.
LEFT_HAND_JOINTS_G1 = [
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
]
RIGHT_HAND_JOINTS_G1 = [
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
]
LEFT_ARM_JOINTS_G1  = [f"left_{j}_joint"  for j in ARM_JOINTS]
RIGHT_ARM_JOINTS_G1 = [f"right_{j}_joint" for j in ARM_JOINTS]

# dex3_hand.csv source columns paired index-for-index with the G1 joint names
# above.  thumb_rotation -> thumb_0, thumb_palm_bend -> thumb_1, thumb_tip_bend
# -> thumb_2; per non-thumb finger, palm_bend -> _0, tip_bend -> _1.
DEX3_LEFT_SRC = [
    "L_actual_thumb_rotation",
    "L_actual_thumb_palm_bend",
    "L_actual_thumb_tip_bend",
    "L_actual_middle_palm_bend",
    "L_actual_middle_tip_bend",
    "L_actual_index_palm_bend",
    "L_actual_index_tip_bend",
]
DEX3_RIGHT_SRC = [
    "R_actual_thumb_rotation",
    "R_actual_thumb_palm_bend",
    "R_actual_thumb_tip_bend",
    "R_actual_index_palm_bend",
    "R_actual_index_tip_bend",
    "R_actual_middle_palm_bend",
    "R_actual_middle_tip_bend",
]

# Mirrors cpp/g1/model/g1Values.hpp::initial_pose (arms zero) plus
# DISABLED_ARM_ELBOW_Q applied in cpp/g1/g1Controller.cpp when a side is disabled.
DISABLED_ARM_ELBOW_Q = 0.8
DISABLED_ARM_POSE = {j: (DISABLED_ARM_ELBOW_Q if j == "elbow" else 0.0) for j in ARM_JOINTS}


def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _bounds_key(joint_name: str) -> str:
    """Strip the trailing '_joint' so we can look up exo/g1 YAML keys."""
    return joint_name[: -len("_joint")] if joint_name.endswith("_joint") else joint_name


def convert_arm_angles(df: pd.DataFrame, joint_names: list[str], exo: dict, g1: dict) -> pd.DataFrame:
    """Replace joint_0…joint_6 bit values with robot angles, in-place copy."""
    df = df.copy()
    for i, name in enumerate(joint_names):
        key = _bounds_key(name)
        exo_j = exo[key]
        g1_j  = g1[key]
        df[name] = df[f"joint_{i}"].apply(
            lambda v: to_robot_angle(
                int(v),
                exo_lower=exo_j["lower"],
                exo_upper=exo_j["upper"],
                skeleton_ref=exo_j["correct_boundary"],
                direction_aligned=exo_j["direction_aligned"],
                g1_lower=g1_j["lower"],
                g1_upper=g1_j["upper"],
            )
        )
    drop = [f"joint_{i}" for i in range(len(joint_names))] + ["timestamp"]
    return df.drop(columns=drop)


def load_dex3_hand(path: Path) -> pd.DataFrame:
    """Load dex3_hand.csv, keep only measured state, rename to G1 joint names."""
    df = pd.read_csv(path)
    keys = ["collection_id", "host_timestamp"]
    out = df[keys].copy()
    pairs = list(zip(DEX3_LEFT_SRC,  LEFT_HAND_JOINTS_G1)) \
          + list(zip(DEX3_RIGHT_SRC, RIGHT_HAND_JOINTS_G1))
    for src, dst in pairs:
        if src not in df.columns:
            raise KeyError(f"Expected column '{src}' in {path}; got {list(df.columns)[:10]}...")
        out[dst] = df[src]
    return out


def load_hand_csv(data_dir: Path) -> pd.DataFrame:
    dex3 = data_dir / "dex3_hand.csv"
    if not dex3.exists():
        raise FileNotFoundError(f"No dex3_hand.csv in {data_dir}")
    return load_dex3_hand(dex3)


def load_measured_arm(path: Path, joint_names: list[str]) -> pd.DataFrame:
    """Load an arm CSV whose joint_0..joint_N are already in robot-frame radians.

    Renames joint_i -> joint_names[i] and drops the device 'timestamp' column.
    """
    df = pd.read_csv(path).copy()
    rename = {f"joint_{i}": name for i, name in enumerate(joint_names)}
    df = df.rename(columns=rename)
    if "timestamp" in df.columns:
        df = df.drop(columns=["timestamp"])
    return df


def synthesize_disabled_arm(reference: pd.DataFrame, joint_names: list[str]) -> pd.DataFrame:
    """Build a DataFrame that mirrors `reference`'s host_timestamp + collection_id,
    with joint columns held steady at the disabled-arm initial pose.
    """
    df = reference[["collection_id", "host_timestamp"]].copy()
    for full_name in joint_names:
        # full_name is e.g. 'left_shoulder_pitch_joint'; strip side + suffix.
        short = _bounds_key(full_name).split("_", 1)[1]
        df[full_name] = DISABLED_ARM_POSE[short]
    return df


def load_arm_csv(data_dir: Path, side: str, joints: list[str], exo: dict, g1: dict) -> pd.DataFrame | None:
    measured = data_dir / f"{side}_measured.csv"
    if measured.exists():
        df = load_measured_arm(measured, joints)
        return None if df.empty else df

    raw = data_dir / f"{side}_arm.csv"
    if raw.exists():
        df = pd.read_csv(raw)
        return None if df.empty else convert_arm_angles(df, joints, exo, g1)

    return None


def load_completed_collection_ids(data_dir: Path) -> set[int] | None:
    """Return the set of collection_ids whose status is 'completed', or None
    if no collection_status.csv is present (treat as keep-all)."""
    path = data_dir / "collection_status.csv"
    if not path.exists():
        return None
    s = pd.read_csv(path)
    return set(s.loc[s["status"] == "completed", "collection_id"].astype(int))


def postprocess(date: str) -> None:
    data_dir = REPO_ROOT / "data" / date

    camera = pd.read_csv(data_dir / "camera.csv")
    hand   = load_hand_csv(data_dir)

    completed = load_completed_collection_ids(data_dir)
    if completed is not None:
        def _keep_completed(df: pd.DataFrame) -> pd.DataFrame:
            return df[df["collection_id"].isin(completed)].reset_index(drop=True)
        camera = _keep_completed(camera)
        hand   = _keep_completed(hand)

    exo = load_yaml(EXO_BOUNDS_PATH)
    g1  = load_yaml(G1_BOUNDS_PATH)

    left  = load_arm_csv(data_dir, "left",  LEFT_ARM_JOINTS_G1,  exo, g1)
    right = load_arm_csv(data_dir, "right", RIGHT_ARM_JOINTS_G1, exo, g1)
    if left is None and right is None:
        raise FileNotFoundError(f"No arm recording (left_arm.csv or right_arm.csv) in {data_dir}")
    if completed is not None:
        if left  is not None: left  = left [left ["collection_id"].isin(completed)].reset_index(drop=True)
        if right is not None: right = right[right["collection_id"].isin(completed)].reset_index(drop=True)
    if left is None:
        left  = synthesize_disabled_arm(right, LEFT_ARM_JOINTS_G1)
    if right is None:
        right = synthesize_disabled_arm(left,  RIGHT_ARM_JOINTS_G1)

    # merge_asof requires both sides sorted by the key
    camera = camera.sort_values("host_timestamp")
    hand   = hand.sort_values("host_timestamp")
    left   = left.sort_values("host_timestamp")
    right  = right.sort_values("host_timestamp")

    merged = pd.merge_asof(camera, left,  on="host_timestamp", by="collection_id", direction="nearest")
    merged = pd.merge_asof(merged, right, on="host_timestamp", by="collection_id", direction="nearest")
    merged = pd.merge_asof(merged, hand,  on="host_timestamp", by="collection_id", direction="nearest")

    if "frame_number" in merged.columns:
        merged = merged.rename(columns={"frame_number": "frame"})
    # downstream LeRobot converters expect ``frame`` to be a filename, not an int.
    if pd.api.types.is_integer_dtype(merged["frame"]):
        merged["frame"] = merged["frame"].map(lambda n: f"frame_{int(n):06d}.png")

    final_cols = (
        META_COLS
        + LEFT_HAND_JOINTS_G1 + RIGHT_HAND_JOINTS_G1
        + LEFT_ARM_JOINTS_G1  + RIGHT_ARM_JOINTS_G1
    )
    merged = merged[final_cols]

    out = data_dir / "data.csv"
    merged.to_csv(out, index=False)
    print(f"Written {len(merged)} rows → {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Postprocess a recorded session.")
    parser.add_argument("date", help="Session folder name, e.g. april_19_16:31")
    postprocess(parser.parse_args().date)
