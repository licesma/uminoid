import argparse
import os
import sys
from pathlib import Path
from typing import Any, Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # add py/postprocess/ to sys.path

import imageio.v3 as iio
import numpy as np
import pandas as pd
from datasets import Dataset

from processor.lerobot.constants import (
    ACTION_COLUMNS,
    CHUNKS_SIZE,
    FRAME_RATE,
    PSI0_DIM,
    ROBOT_TYPE,
    STATE_COLUMNS,
)
from processor.lerobot.metadata import (
    build_features,
    episode_stats_block,
    write_metadata,
)
from processor.lerobot.quality import compute_episode_quality, write_quality_report


def _normalize_columns(df: pd.DataFrame) -> pd.DataFrame:
    """No-op for current Dex3-native CSV format; kept as a hook for future formats."""
    return df


def _pack_episode_matrix(ep_df: pd.DataFrame, col_blocks: List[List[str]]) -> np.ndarray:
    """Shape: (T, PSI0_DIM) — pack the given column blocks into a fixed-layout."""
    T = len(ep_df)
    mat = np.zeros((T, PSI0_DIM), dtype=np.float32)
    offset = 0
    for cols in col_blocks:
        mat[:, offset:offset + len(cols)] = ep_df[cols].to_numpy(dtype=np.float32)
        offset += len(cols)
    return mat


def _write_video(frame_paths: List[Path], out_path: Path) -> None:
    # Format-agnostic: iio.imread auto-detects PNG/JPEG/etc from file contents.
    out_path.parent.mkdir(parents=True, exist_ok=True)
    frames = [iio.imread(str(p)) for p in frame_paths]
    iio.imwrite(
        str(out_path),
        frames,
        fps=FRAME_RATE,
        codec="libx264",
        pixelformat="yuv420p",
    )


def convert_to_lerobot(
    sessions: List[tuple[pd.DataFrame, Path]],
    out_dir: Path,
    instruction: str,
) -> Path:
    """Build a single LeRobot-V2 dataset from N (df, frames_dir) sessions.

    Each session's `collection_id`s define episodes; sessions don't need
    distinct collection_ids because the outer loop assigns globally unique
    `ep_idx` values via the running counter — collisions across sessions
    are structurally impossible.
    """
    state_cols  = [c for block in STATE_COLUMNS  for c in block]
    action_cols = [c for block in ACTION_COLUMNS for c in block]

    work_dir = out_dir
    (work_dir / "data").mkdir(parents=True, exist_ok=True)
    (work_dir / "videos").mkdir(parents=True, exist_ok=True)
    meta_dir = work_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    features = build_features()

    task_index = 0
    tasks = [(task_index, instruction)]

    episodes_meta: List[Dict[str, Any]] = []
    episodes_stats_lines: List[Dict[str, Any]] = []
    quality_records: List[Dict[str, Any]] = []
    ep_idx = 0                # global episode index across all sessions
    dataset_cursor = 0        # global frame cursor across all sessions
    total_frames = 0
    h = w = 0                 # filled from the first session's first frame

    for session_idx, (df, frames_dir) in enumerate(sessions):
        df = _normalize_columns(df)

        if session_idx == 0:
            probe_img = iio.imread(str(frames_dir / df["frame"].iloc[0]))
            h, w = int(probe_img.shape[0]), int(probe_img.shape[1])

        for key in sorted(df["collection_id"].unique().tolist()):
            ep_df = df[df["collection_id"] == key].sort_values("host_timestamp").reset_index(drop=True)

            states  = _pack_episode_matrix(ep_df, STATE_COLUMNS)   # (T, 36) measured
            actions = _pack_episode_matrix(ep_df, ACTION_COLUMNS)  # (T, 36) commanded
            frame_names = ep_df["frame"].tolist()

            q = compute_episode_quality(
                ep_idx, key, ep_df, state_cols, action_cols, n_frames_written=len(states),
            )
            quality_records.append(q)

            frame_paths = [frames_dir / name for name in frame_names]

            n = len(states)
            rows = [
                {
                    "states": states[i].tolist(),
                    "action": actions[i].tolist(),
                    "timestamp": float(i) / FRAME_RATE,
                    "frame_index": i,
                    "episode_index": ep_idx,
                    "index": dataset_cursor + i,
                    "task_index": task_index,
                    "next.done": (i == n - 1),
                }
                for i in range(n)
            ]

            chunk_id = ep_idx // CHUNKS_SIZE
            parquet_path = work_dir / "data" / f"chunk-{chunk_id:03d}" / f"episode_{ep_idx:06d}.parquet"
            video_path = (
                work_dir / "videos" / f"chunk-{chunk_id:03d}" / "egocentric" / f"episode_{ep_idx:06d}.mp4"
            )
            parquet_path.parent.mkdir(parents=True, exist_ok=True)

            ds = Dataset.from_list(rows, features=features)
            tmp_parquet = parquet_path.with_suffix(".parquet.tmp")
            ds.to_parquet(str(tmp_parquet))
            os.replace(tmp_parquet, parquet_path)

            _write_video(frame_paths, video_path)

            episodes_meta.append({
                "episode_index": ep_idx,
                "tasks": [task_index],
                "length": n,
                "dataset_from_index": dataset_cursor,
                "dataset_to_index": dataset_cursor + n - 1,
                "robot_type": ROBOT_TYPE,
                "instruction": instruction,
            })
            episodes_stats_lines.append({
                "episode_index": ep_idx,
                "stats": episode_stats_block(states, actions),
            })
            dataset_cursor += n
            total_frames += n
            ep_idx += 1

    write_metadata(
        meta_dir,
        work_dir,
        total_frames=total_frames,
        episodes_meta=episodes_meta,
        episodes_stats_lines=episodes_stats_lines,
        tasks=tasks,
        h=h,
        w=w,
    )
    write_quality_report(meta_dir, quality_records)
    return work_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True,
                        help="Directory containing data.csv and frames/")
    parser.add_argument("--out-dir", type=Path, required=True,
                        help="Dataset root directly (meta/, data/, videos/ will be created inside)")
    parser.add_argument("--instruction", type=str, default="exoskeleton teleop")
    args = parser.parse_args()

    data_root = args.data_root.expanduser().resolve()
    df = pd.read_csv(data_root / "data.csv")
    frames_dir = data_root / "frames"

    convert_to_lerobot(
        [(df, frames_dir)],
        out_dir=args.out_dir.expanduser().resolve(),
        instruction=args.instruction,
    )


if __name__ == "__main__":
    main()
