#!/usr/bin/env python3
"""Postprocess a recorded session.

For each camera frame (the master timeline), finds the nearest sample by
host_timestamp — within the same collection_id — from left_arm, right_arm,
and inspire_hand.  Raw encoder bit values are converted to robot joint angles
via to_robot_angle.

Usage:
    python postprocess_data.py april_19_16:31
"""

import argparse
from pathlib import Path

import pandas as pd
import yaml

from joints import ARM_JOINTS
from utils.convert_to_radian import to_robot_angle

REPO_ROOT = Path(__file__).parent
EXO_BOUNDS_PATH = REPO_ROOT / "cpp/upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
G1_BOUNDS_PATH  = REPO_ROOT / "cpp/g1/model/upperBodyJointBounds.yaml"

LEFT_JOINTS  = [f"left_{j}"  for j in ARM_JOINTS]
RIGHT_JOINTS = [f"right_{j}" for j in ARM_JOINTS]


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


def postprocess(date: str) -> None:
    data_dir = REPO_ROOT / "data" / date

    camera = pd.read_csv(data_dir / "camera.csv")
    left   = pd.read_csv(data_dir / "left_arm.csv")
    right  = pd.read_csv(data_dir / "right_arm.csv")
    hand   = pd.read_csv(data_dir / "inspire_hand.csv")

    exo = load_yaml(EXO_BOUNDS_PATH)
    g1  = load_yaml(G1_BOUNDS_PATH)

    left  = convert_arm_angles(left,  LEFT_JOINTS,  exo, g1)
    right = convert_arm_angles(right, RIGHT_JOINTS, exo, g1)

    # merge_asof requires both sides sorted by the key
    camera = camera.sort_values("host_timestamp")
    left   = left.sort_values("host_timestamp")
    right  = right.sort_values("host_timestamp")
    hand   = hand.sort_values("host_timestamp")

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
