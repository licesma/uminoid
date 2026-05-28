import shutil
from pathlib import Path

import pandas as pd

from paths import DATA_DIR, FINAL_DATA_DIR, TRAINING_DATA_DIR
from processor.components.status import load_completed_collections
from processor.components.arm import load_arms
from processor.components.hand import load_hand
from processor.components.camera import load_camera
from processor.components.mode import Mode
from processor.components.frame import compress_frames, OUTPUT_DIR
from processor.components.columns import (
    ACTION_JOINT_COLS,
    META_COLS,
    STATE_JOINT_COLS,
    validate_collection_ids,
    validate_columns,
    validate_frames_exist,
)
from processor.lerobot.lerobot import convert_to_lerobot

DATA_CSV = "data.csv"
FRAMES_SUBDIR = "frames"
KEYS = ["collection_id", "host_timestamp"]


class Postprocessor:
    def __init__(
        self,
        episode_names: list[str],
        name: str,
        mode: Mode,
        instruction: str,
        save_final: bool = False,
    ):
        self.episode_paths: list[Path] = [DATA_DIR / n for n in episode_names]
        self.name: str = name
        self.mode: Mode = mode
        self.instruction: str = instruction
        self.save_final: bool = save_final

    def run(self) -> None:
        sessions: list[tuple[pd.DataFrame, Path]] = []

        for ep_path in self.episode_paths:
            status_df = load_completed_collections(ep_path)
            compress_frames(ep_path)
            left_arm, right_arm = load_arms(ep_path, status_df, self.mode)
            hand = load_hand(ep_path, status_df)
            camera = load_camera(ep_path, status_df)

            episode_df = self._create_episode(camera, left_arm, right_arm, hand)
            validate_columns(episode_df)
            validate_collection_ids(episode_df)
            validate_frames_exist(episode_df, ep_path / OUTPUT_DIR)

            out = ep_path / DATA_CSV
            episode_df.to_csv(out, index=False)
            print(f"Wrote {len(episode_df)} rows -> {out}")

            sessions.append((episode_df, ep_path / OUTPUT_DIR))

            if self.save_final:
                self._save_final(episode_df, ep_path)

        out_dir = TRAINING_DATA_DIR / self.name
        convert_to_lerobot(sessions, out_dir, self.instruction)
        print(f"Wrote LeRobot dataset ({len(sessions)} session(s)) -> {out_dir}")

    def _create_episode(self, camera: pd.DataFrame, left_arm: pd.DataFrame,
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

        # Final ordering: meta, then all state cols (hand+arm) then all action
        # cols, matching what STATE_COLUMNS / ACTION_COLUMNS expect downstream.
        final_cols = META_COLS + STATE_JOINT_COLS + ACTION_JOINT_COLS
        return merged[final_cols]

    def _save_final(self, episode_df: pd.DataFrame, ep_path: Path) -> None:
        dst = FINAL_DATA_DIR / ep_path.name
        if dst.exists():
            shutil.rmtree(dst)
        frames_dst = dst / FRAMES_SUBDIR
        frames_dst.mkdir(parents=True)

        episode_df.to_csv(dst / DATA_CSV, index=False)

        png_src = ep_path / OUTPUT_DIR
        for frame_name in episode_df["frame"].astype(str).unique():
            src = png_src / frame_name
            if src.is_file():
                shutil.copy2(src, frames_dst / frame_name)
