#!/usr/bin/env python3
"""Postprocess a recorded session.

For each camera frame (the master timeline), finds the nearest sample by
host_timestamp — within the same collection_id — from left_arm, right_arm,
and the hand recording.  Raw arm encoder bit values are converted to robot
joint angles via to_robot_angle.

The hand recording is auto-detected: a session may contain either
``inspire_hand.csv`` or ``dex3_hand.csv`` (not both).  Whichever is present is
merged into the output as-is so the column schema follows the native hardware.

Each arm side is also auto-detected per-file:
- ``{side}_measured.csv`` — joint_0..joint_N already in robot-frame radians; the
  bit-to-angle conversion is skipped.
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

LEFT_JOINTS  = [f"left_{j}"  for j in ARM_JOINTS]
RIGHT_JOINTS = [f"right_{j}" for j in ARM_JOINTS]

# Mirrors cpp/g1/model/g1Values.hpp::initial_pose (arms zero) plus
# DISABLED_ARM_ELBOW_Q applied in cpp/g1/g1Controller.cpp when a side is disabled.
DISABLED_ARM_ELBOW_Q = 0.8
DISABLED_ARM_POSE = {j: (DISABLED_ARM_ELBOW_Q if j == "elbow" else 0.0) for j in ARM_JOINTS}


def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def convert_arm_angles(df: pd.DataFrame, joint_names: list[str], exo: dict, g1: dict) -> pd.DataFrame:
    """Replace joint_0…joint_6 bit values with robot angles, in-place copy."""
    df = df.copy()
    for i, name in enumerate(joint_names):
        exo_j = exo[name]
        g1_j  = g1[name]
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


def load_hand_csv(data_dir: Path) -> pd.DataFrame:
    inspire = data_dir / "inspire_hand.csv"
    dex3    = data_dir / "dex3_hand.csv"
    if inspire.exists() and dex3.exists():
        raise RuntimeError(
            f"Both inspire_hand.csv and dex3_hand.csv exist in {data_dir}; "
            "session must contain exactly one hand recording."
        )
    if inspire.exists():
        return pd.read_csv(inspire)
    if dex3.exists():
        return pd.read_csv(dex3)
    raise FileNotFoundError(
        f"No hand recording (inspire_hand.csv or dex3_hand.csv) in {data_dir}"
    )


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
        short = full_name.split("_", 1)[1]  # strip 'left_' / 'right_'
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


def postprocess(date: str) -> None:
    data_dir = REPO_ROOT / "data" / date

    camera = pd.read_csv(data_dir / "camera.csv")
    hand   = load_hand_csv(data_dir)

    exo = load_yaml(EXO_BOUNDS_PATH)
    g1  = load_yaml(G1_BOUNDS_PATH)

    left  = load_arm_csv(data_dir, "left",  LEFT_JOINTS,  exo, g1)
    right = load_arm_csv(data_dir, "right", RIGHT_JOINTS, exo, g1)
    if left is None and right is None:
        raise FileNotFoundError(f"No arm recording (left_arm.csv or right_arm.csv) in {data_dir}")
    if left is None:
        left  = synthesize_disabled_arm(right, LEFT_JOINTS)
    if right is None:
        right = synthesize_disabled_arm(left,  RIGHT_JOINTS)

    # merge_asof requires both sides sorted by the key
    camera = camera.sort_values("host_timestamp")
    hand   = hand.sort_values("host_timestamp")
    left   = left.sort_values("host_timestamp")
    right  = right.sort_values("host_timestamp")

    merged = pd.merge_asof(camera, left,  on="host_timestamp", by="collection_id", direction="nearest")
    merged = pd.merge_asof(merged, right, on="host_timestamp", by="collection_id", direction="nearest")
    merged = pd.merge_asof(merged, hand,  on="host_timestamp", by="collection_id", direction="nearest")

    out = data_dir / "data.csv"
    merged.to_csv(out, index=False)
    print(f"Written {len(merged)} rows → {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Postprocess a recorded session.")
    parser.add_argument("date", help="Session folder name, e.g. april_19_16:31")
    postprocess(parser.parse_args().date)
