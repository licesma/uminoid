"""Convert exoskeleton teleop CSV+frames into Psi-0's LeRobot-V2 HF dataset.

Input layout (under --data-root, e.g. /data/jiageng/dataset/exoskeleton-dataset/init/final):
    data.csv            # one row per camera frame
    frames/
        frame_XXXXXX.png   # 480x640 RGB

`data.csv` columns (Dex3-native format, joint angles in radians):
    collection_id, frame, host_timestamp,
    left_hand_thumb_0_joint, left_hand_thumb_1_joint, left_hand_thumb_2_joint,
    left_hand_middle_0_joint, left_hand_middle_1_joint,
    left_hand_index_0_joint,  left_hand_index_1_joint,
    right_hand_thumb_0_joint, right_hand_thumb_1_joint, right_hand_thumb_2_joint,
    right_hand_index_0_joint,  right_hand_index_1_joint,
    right_hand_middle_0_joint, right_hand_middle_1_joint,
    left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint,
    left_elbow_joint, left_wrist_roll_joint, left_wrist_pitch_joint, left_wrist_yaw_joint,
    right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint,
    right_elbow_joint, right_wrist_roll_joint, right_wrist_pitch_joint, right_wrist_yaw_joint,

Hand-column order matches Dex3-1 joint enums (real/teleop/robot_control/robot_hand_unitree.py):
    left:  thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1
    right: thumb_0, thumb_1, thumb_2, index_0,  index_1,  middle_0, middle_1

Psi-0 G1 36-dim layout (state and action share the same packing):
    [ 0: 7]  left_hand   — Dex3 left joints in enum order (see above)               (7)
    [ 7:14]  right_hand  — Dex3 right joints in enum order (see above)              (7)
    [14:21]  left_arm    — shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw     (7)
    [21:28]  right_arm   — same ordering                                            (7)
    [28:32]  torso       — roll, pitch, yaw, height                                 (zeros; exo data has none)
    [32:36]  base        — vx, vy, vyaw, target_yaw                                 (zeros; exo data has none)

Action convention:
    We only have proprioceptive state in the CSV (no explicit commands), so this script
    uses `action[t] = state[t+1]` — i.e. the action is the state the exoskeleton will
    drive the body to in the next frame. The final frame of every episode is therefore
    dropped (no t+1).

Output (at --out-dir/):
    meta/{info.json, episodes.jsonl, tasks.jsonl, episodes_stats.jsonl,
          stats_psi0.json, stats.json, quality_report.json}
    data/chunk-000/episode_XXXXXX.parquet
    videos/chunk-000/egocentric/episode_XXXXXX.mp4   # H.264 yuv420p 30 fps

Data-quality diagnostics (written into meta/quality_report.json):
    - `host_timestamp` dt stats per episode (min/median/p95/max in ms)
    - count of dt gaps > 100ms and > 200ms (expected is ~33ms at 30Hz)
    - count of consecutive-identical joint/hand rows — proxy for stale async sync,
      where the joint/hand stream had no fresh reading between two image frames
    - overall `flagged_episodes` list (episodes with dt_max_ms > --flag-dt-ms OR
      dup_ratio > --flag-dup-ratio)

Note on async sync: the producer-side CSV is 1 row per image frame with a single
`host_timestamp`, i.e. the image/joint/hand sync has already been done upstream.
Without the separate raw joint/hand logs we cannot redo the matching; the quality
report is the best we can do with what's in the CSV.

Usage:
    python scripts/data/exoskeleton_to_psi0_lerobot.py \\
        --data-root ~/Downloads/may_06_12_35 \\
        --out-dir   $DATA_HOME/exoskeleton-dataset/lerobot/pick_marker_pen_place_box \\
        --instruction "pick up the blue marker pen and place it into the box"
"""

import argparse
import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

import imageio.v3 as iio
import numpy as np
import pandas as pd
from datasets import Dataset, Features, Sequence as DSSequence, Value

CODE_VERSION = "v2.1"
FPS = 30
CHUNKS_SIZE = 1000

LEFT_HAND_COLS = [
    "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    "left_hand_index_0_joint",  "left_hand_index_1_joint",
]
RIGHT_HAND_COLS = [
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
    "right_hand_index_0_joint",  "right_hand_index_1_joint",
    "right_hand_middle_0_joint", "right_hand_middle_1_joint",
]
LEFT_ARM_COLS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
]
RIGHT_ARM_COLS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


def _normalize_columns(df: pd.DataFrame) -> pd.DataFrame:
    """No-op for current Dex3-native CSV format; kept as a hook for future formats."""
    return df


HAND_SLOTS = 7           # G1 Dex3 per-hand DOFs
ARM_SLOTS = 7
TORSO_SLOTS = 4
BASE_SLOTS = 4
PSI0_DIM = 2 * HAND_SLOTS + 2 * ARM_SLOTS + TORSO_SLOTS + BASE_SLOTS  # 36


@dataclass
class InfoDict:
    codebase_version: str
    robot_type: str
    total_episodes: int
    total_frames: int
    total_tasks: int
    total_videos: int
    total_chunks: int
    chunks_size: int
    fps: int
    data_path: str
    video_path: str
    features: Dict[str, Any]


def _build_features() -> Features:
    return Features({
        "states": DSSequence(Value("float32")),
        "action": DSSequence(Value("float32")),
        "timestamp": Value("float32"),
        "frame_index": Value("int64"),
        "episode_index": Value("int64"),
        "index": Value("int64"),
        "task_index": Value("int64"),
        "next.done": Value("bool"),
    })


def _pack_psi0_vector(row_cols: Dict[str, float]) -> np.ndarray:
    """Pack one CSV row's joint values into the 36-dim Psi-0 G1 layout."""
    vec = np.zeros(PSI0_DIM, dtype=np.float32)
    for i, c in enumerate(LEFT_HAND_COLS):
        vec[i] = row_cols[c]
    for i, c in enumerate(RIGHT_HAND_COLS):
        vec[HAND_SLOTS + i] = row_cols[c]
    for i, c in enumerate(LEFT_ARM_COLS):
        vec[2 * HAND_SLOTS + i] = row_cols[c]
    for i, c in enumerate(RIGHT_ARM_COLS):
        vec[2 * HAND_SLOTS + ARM_SLOTS + i] = row_cols[c]
    # torso[28:32] and base[32:36] stay zero — no corresponding exoskeleton signal.
    return vec


def _pack_episode_matrix(ep_df: pd.DataFrame) -> np.ndarray:
    """Shape: (T, 36) — state vector per row, in Psi-0 G1 layout."""
    T = len(ep_df)
    mat = np.zeros((T, PSI0_DIM), dtype=np.float32)
    col_blocks = [
        (LEFT_HAND_COLS,  0),
        (RIGHT_HAND_COLS, HAND_SLOTS),
        (LEFT_ARM_COLS,   2 * HAND_SLOTS),
        (RIGHT_ARM_COLS,  2 * HAND_SLOTS + ARM_SLOTS),
    ]
    for cols, offset in col_blocks:
        mat[:, offset:offset + len(cols)] = ep_df[cols].to_numpy(dtype=np.float32)
    return mat


def _write_video(frame_paths: List[Path], out_path: Path, fps: int) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    frames = [iio.imread(str(p)) for p in frame_paths]
    iio.imwrite(
        str(out_path),
        frames,
        fps=fps,
        codec="libx264",
        pixelformat="yuv420p",
    )


def _episode_stats_block(
    states: np.ndarray, actions: np.ndarray, fps: int,
) -> Dict[str, Any]:
    n = len(states)
    return {
        "action": {
            "min": actions.min(axis=0).tolist(),
            "max": actions.max(axis=0).tolist(),
            "mean": actions.mean(axis=0).tolist(),
            "std": actions.std(axis=0).tolist(),
            "count": [n],
        },
        "states": {
            "min": states.min(axis=0).tolist(),
            "max": states.max(axis=0).tolist(),
            "mean": states.mean(axis=0).tolist(),
            "std": states.std(axis=0).tolist(),
            "count": [n],
        },
        "timestamp": {
            "min": [0.0],
            "max": [(n - 1) / fps],
            "mean": [((n - 1) / 2) / fps],
            "std": [n / (2 * fps * math.sqrt(3))],
            "count": [n],
        },
    }


def _global_stats(parquet_paths: List[Path]) -> Dict[str, Dict[str, List[float]]]:
    buffers: Dict[str, List[np.ndarray]] = {"action": [], "states": []}
    for p in parquet_paths:
        df = pd.read_parquet(p)
        for key in buffers:
            buffers[key].append(
                np.vstack([np.asarray(x, dtype=np.float32) for x in df[key]])
            )
    stats: Dict[str, Dict[str, List[float]]] = {}
    for key, arrs in buffers.items():
        arr = np.concatenate(arrs, axis=0)
        stats[key] = {
            "mean": arr.mean(axis=0).tolist(),
            "std": arr.std(axis=0).tolist(),
            "min": arr.min(axis=0).tolist(),
            "max": arr.max(axis=0).tolist(),
            "q01": np.quantile(arr, 0.01, axis=0).tolist(),
            "q99": np.quantile(arr, 0.99, axis=0).tolist(),
        }
    return stats


def convert(
    data_root: Path,
    out_dir: Path,
    *,
    instruction: str,
    fps: int,
    chunks_size: int,
    robot_type: str,
    flag_dt_ms: float,
    flag_dup_ratio: float,
    exclude_collection_ids: List[int],
) -> Path:
    csv_path = data_root / "data.csv"
    frames_dir = data_root / "frames"
    assert csv_path.is_file(), f"missing {csv_path}"
    assert frames_dir.is_dir(), f"missing {frames_dir}"

    df = _normalize_columns(pd.read_csv(csv_path))
    joint_cols = [*LEFT_HAND_COLS, *RIGHT_HAND_COLS, *LEFT_ARM_COLS, *RIGHT_ARM_COLS]
    missing_cols = [
        c for c in ["collection_id", "frame", "host_timestamp", *joint_cols]
        if c not in df.columns
    ]
    if missing_cols:
        raise ValueError(f"CSV missing expected columns: {missing_cols}")

    work_dir = out_dir
    (work_dir / "data").mkdir(parents=True, exist_ok=True)
    (work_dir / "videos").mkdir(parents=True, exist_ok=True)
    meta_dir = work_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    features = _build_features()
    all_keys = sorted(df["collection_id"].unique().tolist())
    excluded = set(exclude_collection_ids)
    episode_keys = [k for k in all_keys if int(k) not in excluded]
    skipped = [k for k in all_keys if int(k) in excluded]
    if skipped:
        print(f"excluding {len(skipped)} collection_id(s) by request: {skipped}")

    task_index = 0
    tasks = [(task_index, instruction)]

    episodes_meta: List[Dict[str, Any]] = []
    episodes_stats_lines: List[Dict[str, Any]] = []
    quality_records: List[Dict[str, Any]] = []
    dataset_cursor = 0
    total_frames = 0

    # Probe first frame for resolution.
    probe_img = iio.imread(str(frames_dir / df["frame"].iloc[0]))
    h, w = int(probe_img.shape[0]), int(probe_img.shape[1])

    expected_dt_ms = 1000.0 / fps  # ~33.3 ms at 30Hz
    for ep_idx, key in enumerate(episode_keys):
        ep_df = df[df["collection_id"] == key].sort_values("host_timestamp").reset_index(drop=True)
        if len(ep_df) < 2:
            print(f"[skip] collection_id={key} has {len(ep_df)} frame(s); need ≥2 for action=next")
            continue

        state_mat = _pack_episode_matrix(ep_df)        # (T, 36)
        # action[t] = state[t+1] — last frame has no successor, drop it.
        states = state_mat[:-1]
        actions = state_mat[1:]
        frame_names = ep_df["frame"].tolist()[:-1]

        # --- quality diagnostics -------------------------------------------
        dt_ms = np.diff(ep_df["host_timestamp"].to_numpy().astype(np.float64)) / 1000.0
        # Consecutive identical joint/hand rows: a fingerprint of stale async
        # sync (the joint/hand reader had no fresh reading between two frames,
        # so the producer reused the previous sample). Harmless in small doses,
        # but a high ratio means the async streams fell behind the camera.
        jmat = ep_df[joint_cols].to_numpy()
        dup_mask = np.all(jmat[1:] == jmat[:-1], axis=1)
        dup_count = int(dup_mask.sum())
        dup_ratio = dup_count / max(1, len(dup_mask))

        flags = []
        if dt_ms.max() > flag_dt_ms:
            flags.append("large_timestamp_gap")
        if dup_ratio > flag_dup_ratio:
            flags.append("stale_async_sync")

        q = {
            "episode_index": ep_idx,
            "collection_id": int(key),
            "n_frames_csv": int(len(ep_df)),
            "n_frames_written": int(len(states)),
            "dt_min_ms": round(float(dt_ms.min()), 2),
            "dt_median_ms": round(float(np.median(dt_ms)), 2),
            "dt_p95_ms": round(float(np.quantile(dt_ms, 0.95)), 2),
            "dt_max_ms": round(float(dt_ms.max()), 2),
            "n_gaps_gt_100ms": int((dt_ms > 100.0).sum()),
            "n_gaps_gt_200ms": int((dt_ms > 200.0).sum()),
            "consecutive_dup_joint_rows": dup_count,
            "consecutive_dup_joint_ratio": round(dup_ratio, 4),
            "flags": flags,
        }
        quality_records.append(q)
        # ------------------------------------------------------------------

        frame_paths = [frames_dir / name for name in frame_names]
        missing = [p for p in frame_paths if not p.is_file()]
        if missing:
            raise FileNotFoundError(f"missing {len(missing)} frames for episode {ep_idx}, e.g. {missing[0]}")

        n = len(states)
        rows = [
            {
                "states": states[i].tolist(),
                "action": actions[i].tolist(),
                "timestamp": float(i) / fps,
                "frame_index": i,
                "episode_index": ep_idx,
                "index": dataset_cursor + i,
                "task_index": task_index,
                "next.done": (i == n - 1),
            }
            for i in range(n)
        ]

        chunk_id = ep_idx // chunks_size
        parquet_path = work_dir / "data" / f"chunk-{chunk_id:03d}" / f"episode_{ep_idx:06d}.parquet"
        video_path = (
            work_dir / "videos" / f"chunk-{chunk_id:03d}" / "egocentric" / f"episode_{ep_idx:06d}.mp4"
        )
        parquet_path.parent.mkdir(parents=True, exist_ok=True)

        ds = Dataset.from_list(rows, features=features)
        tmp_parquet = parquet_path.with_suffix(".parquet.tmp")
        ds.to_parquet(str(tmp_parquet))
        os.replace(tmp_parquet, parquet_path)

        _write_video(frame_paths, video_path, fps=fps)

        episodes_meta.append({
            "episode_index": ep_idx,
            "tasks": [task_index],
            "length": n,
            "dataset_from_index": dataset_cursor,
            "dataset_to_index": dataset_cursor + n - 1,
            "robot_type": robot_type,
            "instruction": instruction,
        })
        episodes_stats_lines.append({
            "episode_index": ep_idx,
            "stats": _episode_stats_block(states, actions, fps=fps),
        })
        dataset_cursor += n
        total_frames += n
        flag_str = f"  ⚠ {','.join(flags)}" if flags else ""
        print(
            f"[ok] episode {ep_idx:06d}  collection={key}  frames={n}  "
            f"dt_max={q['dt_max_ms']:.1f}ms  dup={dup_count}{flag_str}"
        )

    if total_frames == 0:
        raise RuntimeError("no episodes produced — check input data")

    features_meta = {
        "observation.images.egocentric": {
            "dtype": "video",
            "shape": [h, w, 3],
            "names": ["height", "width", "channel"],
            "video_info": {
                "video.fps": float(fps),
                "video.codec": "h264",
                "video.pix_fmt": "yuv420p",
                "video.is_depth_map": False,
                "has_audio": False,
            },
        },
        "states": {"dtype": "float32", "shape": [PSI0_DIM]},
        "action": {"dtype": "float32", "shape": [PSI0_DIM]},
        "timestamp": {"dtype": "float32", "shape": [1]},
        "frame_index": {"dtype": "int64", "shape": [1]},
        "episode_index": {"dtype": "int64", "shape": [1]},
        "index": {"dtype": "int64", "shape": [1]},
        "next.done": {"dtype": "bool", "shape": [1]},
        "task_index": {"dtype": "int64", "shape": [1]},
    }
    num_episodes = len(episodes_meta)
    info = InfoDict(
        codebase_version=CODE_VERSION,
        robot_type=robot_type,
        total_episodes=num_episodes,
        total_frames=total_frames,
        total_tasks=len(tasks),
        total_videos=num_episodes,
        total_chunks=math.ceil(num_episodes / chunks_size),
        chunks_size=chunks_size,
        fps=fps,
        data_path="data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        video_path="videos/chunk-{episode_chunk:03d}/egocentric/episode_{episode_index:06d}.mp4",
        features=features_meta,
    )
    (meta_dir / "info.json").write_text(json.dumps(asdict(info), indent=4))

    with open(meta_dir / "episodes.jsonl", "w") as f:
        for row in episodes_meta:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    with open(meta_dir / "tasks.jsonl", "w") as f:
        for ti, instr in tasks:
            slug = instr.replace(" ", "_")
            f.write(json.dumps({
                "task_index": ti,
                "task": f"default/{slug}",
                "category": "default",
                "description": instr,
            }, ensure_ascii=False) + "\n")

    with open(meta_dir / "episodes_stats.jsonl", "w") as f:
        for row in episodes_stats_lines:
            f.write(json.dumps(row) + "\n")

    parquet_paths = sorted((work_dir / "data").rglob("episode_*.parquet"))
    stats = _global_stats(parquet_paths)
    with open(meta_dir / "stats_psi0.json", "w") as f:
        json.dump(stats, f, indent=2)
    with open(meta_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    flagged = [q for q in quality_records if q["flags"]]
    report = {
        "fps_assumed": fps,
        "expected_dt_ms": expected_dt_ms,
        "flag_dt_ms": flag_dt_ms,
        "flag_dup_ratio": flag_dup_ratio,
        "n_episodes_total": len(quality_records),
        "n_episodes_flagged": len(flagged),
        "flagged_episode_indices": [q["episode_index"] for q in flagged],
        "flagged_collection_ids": [q["collection_id"] for q in flagged],
        "per_episode": quality_records,
    }
    with open(meta_dir / "quality_report.json", "w") as f:
        json.dump(report, f, indent=2)

    print(f"\n✅ wrote {num_episodes} episodes, {total_frames} frames to {work_dir}")
    dt_max_all = max(q["dt_max_ms"] for q in quality_records)
    mean_fps = 1000.0 / np.mean([q["dt_median_ms"] for q in quality_records])
    print(
        f"timing: median-dt-based fps ≈ {mean_fps:.2f} (assumed {fps}); "
        f"worst intra-ep gap = {dt_max_all:.1f} ms"
    )
    if flagged:
        print(f"\n⚠ {len(flagged)} episode(s) flagged for quality review:")
        for q in flagged:
            print(
                f"  ep={q['episode_index']:>4}  cid={q['collection_id']:>3}  "
                f"n={q['n_frames_written']:>3}  "
                f"dt_max={q['dt_max_ms']:>8.1f}ms  "
                f"dup={q['consecutive_dup_joint_rows']}/{q['n_frames_csv']-1} "
                f"({q['consecutive_dup_joint_ratio']:.1%})  "
                f"flags={q['flags']}"
            )
    else:
        print("no episodes flagged — all gaps within tolerance.")
    print(f"full per-episode quality report: {meta_dir / 'quality_report.json'}")
    return work_dir


def main():
    # Resolve repo paths so we can take a bare episode name and write to a
    # sibling reference-output dir for easy diffing against the new pipeline.
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from paths import FINAL_DATA_DIR, ROOT_DIR
    reference_dir = ROOT_DIR / "training_data_reference"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_name",
                        help="Episode dir under final_data/, e.g. may_25_18:28")
    parser.add_argument("--instruction", type=str, default="exoskeleton teleop")
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--chunks-size", type=int, default=CHUNKS_SIZE)
    parser.add_argument("--robot-type", type=str, default="exoskeleton")
    parser.add_argument("--flag-dt-ms", type=float, default=100.0,
                        help="Flag episode if any host_timestamp gap exceeds this many ms")
    parser.add_argument("--flag-dup-ratio", type=float, default=0.10,
                        help="Flag episode if fraction of consecutive-identical joint rows exceeds this")
    parser.add_argument("--exclude-collection-ids", type=str, default="",
                        help="Comma-separated collection_id values to drop entirely (e.g. '51,73')")
    args = parser.parse_args()

    exclude_ids = [int(x) for x in args.exclude_collection_ids.split(",") if x.strip()]

    convert(
        data_root=(FINAL_DATA_DIR / args.episode_name).resolve(),
        out_dir=(reference_dir / args.episode_name).resolve(),
        instruction=args.instruction,
        fps=args.fps,
        chunks_size=args.chunks_size,
        robot_type=args.robot_type,
        flag_dt_ms=args.flag_dt_ms,
        flag_dup_ratio=args.flag_dup_ratio,
        exclude_collection_ids=exclude_ids,
    )


if __name__ == "__main__":
    main()
