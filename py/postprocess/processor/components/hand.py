from pathlib import Path

import pandas as pd

from helpers.error_check import ensure
from processor.components.status import filter_completed

HAND_FILE = "dex3_hand.csv"
META_COLS = ["collection_id", "host_timestamp"]

# dex3_hand.csv carries cmd/actual/force/pressure in one stream. We keep only the
# measured (*_actual_*) state and rename the dex3 physical joint names to G1 hand
# joint names. Column order follows the firmware wire layout (see
# cpp/hand_retarget/dex3/dex3_values.hpp) -- note the asymmetric L middle/index
# vs R index/middle ordering.
DEX3_SRC_TO_G1 = {
    "L_actual_thumb_rotation":   "left_hand_thumb_0_joint",
    "L_actual_thumb_palm_bend":  "left_hand_thumb_1_joint",
    "L_actual_thumb_tip_bend":   "left_hand_thumb_2_joint",
    "L_actual_middle_palm_bend": "left_hand_middle_0_joint",
    "L_actual_middle_tip_bend":  "left_hand_middle_1_joint",
    "L_actual_index_palm_bend":  "left_hand_index_0_joint",
    "L_actual_index_tip_bend":   "left_hand_index_1_joint",
    "R_actual_thumb_rotation":   "right_hand_thumb_0_joint",
    "R_actual_thumb_palm_bend":  "right_hand_thumb_1_joint",
    "R_actual_thumb_tip_bend":   "right_hand_thumb_2_joint",
    "R_actual_index_palm_bend":  "right_hand_index_0_joint",
    "R_actual_index_tip_bend":   "right_hand_index_1_joint",
    "R_actual_middle_palm_bend": "right_hand_middle_0_joint",
    "R_actual_middle_tip_bend":  "right_hand_middle_1_joint",
}


def load_hand(episode_path: Path, status_df: pd.DataFrame) -> pd.DataFrame:
    path = episode_path / HAND_FILE
    ensure(path.exists(), f"{path} does not exist")
    df = pd.read_csv(path)

    out = df[META_COLS].copy()
    for src, dst in DEX3_SRC_TO_G1.items():
        ensure(src in df.columns, f"Expected column '{src}' in {path}")
        out[dst] = df[src]

    return filter_completed(out, status_df)
