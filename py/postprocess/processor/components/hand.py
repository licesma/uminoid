from pathlib import Path

import pandas as pd

from helpers.error_check import ensure
from processor.components.status import filter_completed

from processor.lerobot.constants import ACTION_SUFFIX, STATE_SUFFIX

HAND_FILE = "dex3_hand.csv"
META_COLS = ["collection_id", "host_timestamp"]


_DEX3_PAIRS = [
    ("L_{}_thumb_rotation",   "left_hand_thumb_0_joint"),
    ("L_{}_thumb_palm_bend",  "left_hand_thumb_1_joint"),
    ("L_{}_thumb_tip_bend",   "left_hand_thumb_2_joint"),
    ("L_{}_middle_palm_bend", "left_hand_middle_0_joint"),
    ("L_{}_middle_tip_bend",  "left_hand_middle_1_joint"),
    ("L_{}_index_palm_bend",  "left_hand_index_0_joint"),
    ("L_{}_index_tip_bend",   "left_hand_index_1_joint"),
    ("R_{}_thumb_rotation",   "right_hand_thumb_0_joint"),
    ("R_{}_thumb_palm_bend",  "right_hand_thumb_1_joint"),
    ("R_{}_thumb_tip_bend",   "right_hand_thumb_2_joint"),
    ("R_{}_index_palm_bend",  "right_hand_index_0_joint"),
    ("R_{}_index_tip_bend",   "right_hand_index_1_joint"),
    ("R_{}_middle_palm_bend", "right_hand_middle_0_joint"),
    ("R_{}_middle_tip_bend",  "right_hand_middle_1_joint"),
]

DEX3_STATE_MAP  = {src.format("actual"): dst + STATE_SUFFIX  for src, dst in _DEX3_PAIRS}
DEX3_ACTION_MAP = {src.format("cmd"):    dst + ACTION_SUFFIX for src, dst in _DEX3_PAIRS}


def load_hand(episode_path: Path, status_df: pd.DataFrame) -> pd.DataFrame:
    path = episode_path / HAND_FILE
    ensure(path.exists(), f"{path} does not exist")
    df = pd.read_csv(path)

    out = df[META_COLS].copy()
    for src_to_dst in (DEX3_STATE_MAP, DEX3_ACTION_MAP):
        for src, dst in src_to_dst.items():
            ensure(src in df.columns, f"Expected column '{src}' in {path}")
            out[dst] = df[src]

    return filter_completed(out, status_df)
