import shutil
from pathlib import Path

import pandas as pd

from paths import DATA_DIR, FINAL_DATA_DIR
from processor.components.status import load_completed_collections
from processor.components.arm import load_arms
from processor.components.hand import load_hand
from processor.components.camera import load_camera
from processor.components.mode import Mode
from processor.components.frame import compress_frames, OUTPUT_DIR

DATA_CSV = "data.csv"
FRAMES_SUBDIR = "frames"
KEYS = ["collection_id", "host_timestamp"]
META_COLS = ["collection_id", "frame", "host_timestamp"]


class Postprocessor:
    def __init__(self, episode_name: str, mode: Mode):
        self.episode_path: Path = DATA_DIR / episode_name
        self.mode: Mode = mode
        self.status_df: pd.DataFrame = load_completed_collections(self.episode_path)

    def run(self) -> None:
        compress_frames(self.episode_path)
        left_arm, right_arm = load_arms(self.episode_path, self.status_df, self.mode)
        hand = load_hand(self.episode_path, self.status_df)
        camera = load_camera(self.episode_path, self.status_df)

        merged = self._merge(camera, left_arm, right_arm, hand)
        out = self.episode_path / DATA_CSV
        merged.to_csv(out, index=False)
        print(f"Wrote {len(merged)} rows -> {out}")

        self._save_final(merged)

    def _merge(self, camera: pd.DataFrame, left_arm: pd.DataFrame,
               right_arm: pd.DataFrame, hand: pd.DataFrame) -> pd.DataFrame:
        """Align every stream to the camera timeline by nearest host_timestamp
        within the same collection"""
        camera, left_arm, right_arm, hand = (
            df.sort_values("host_timestamp") for df in (camera, left_arm, right_arm, hand)
        )
        merged = camera
        for df in (left_arm, right_arm, hand):
            merged = pd.merge_asof(merged, df, on="host_timestamp", by="collection_id", direction="nearest")

        
        merged["frame"] = merged["frame_number"].map(lambda n: f"frame_{int(n):06d}.png")

        def without_keys(df: pd.DataFrame) -> list[str]:
            return [c for c in df.columns if c not in KEYS]

        final_cols = META_COLS + without_keys(hand) + without_keys(left_arm) + without_keys(right_arm)
        return merged[final_cols]

    def _save_final(self, merged: pd.DataFrame) -> None:
        dst = FINAL_DATA_DIR / self.episode_path.name
        if dst.exists():
            shutil.rmtree(dst)
        frames_dst = dst / FRAMES_SUBDIR
        frames_dst.mkdir(parents=True)

        merged.to_csv(dst / DATA_CSV, index=False)

        png_src = self.episode_path / OUTPUT_DIR
        for frame_name in merged["frame"].astype(str).unique():
            src = png_src / frame_name
            if src.is_file():
                shutil.copy2(src, frames_dst / frame_name)
