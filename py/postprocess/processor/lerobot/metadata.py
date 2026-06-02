import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pandas as pd
from datasets import Features, Sequence as DSSequence, Value

from processor.lerobot.constants import (
    CHUNKS_SIZE,
    CODE_VERSION,
    FRAME_RATE,
    PSI0_DIM,
    ROBOT_TYPE,
)


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


def build_features() -> Features:
    """Parquet feature schema written into each per-episode parquet."""
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


def episode_stats_block(states: np.ndarray, actions: np.ndarray) -> Dict[str, Any]:
    """Per-episode min/max/mean/std for state, action, timestamp."""
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
            "max": [(n - 1) / FRAME_RATE],
            "mean": [((n - 1) / 2) / FRAME_RATE],
            "std": [n / (2 * FRAME_RATE * math.sqrt(3))],
            "count": [n],
        },
    }


def _compute_global_stats(parquet_paths: List[Path]) -> Dict[str, Dict[str, List[float]]]:
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


def _features_meta(h: int, w: int) -> Dict[str, Any]:
    return {
        "observation.images.egocentric": {
            "dtype": "video",
            "shape": [h, w, 3],
            "names": ["height", "width", "channel"],
            "video_info": {
                "video.fps": float(FRAME_RATE),
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


def write_metadata(
    meta_dir: Path,
    work_dir: Path,
    *,
    total_frames: int,
    episodes_meta: List[Dict[str, Any]],
    episodes_stats_lines: List[Dict[str, Any]],
    tasks: List[Tuple[int, str]],
    h: int,
    w: int,
) -> None:
    """Emit every file under `meta/` except quality_report.json (owned by quality.py)."""
    num_episodes = len(episodes_meta)
    info = InfoDict(
        codebase_version=CODE_VERSION,
        robot_type=ROBOT_TYPE,
        total_episodes=num_episodes,
        total_frames=total_frames,
        total_tasks=len(tasks),
        total_videos=num_episodes,
        total_chunks=math.ceil(num_episodes / CHUNKS_SIZE),
        chunks_size=CHUNKS_SIZE,
        fps=FRAME_RATE,
        data_path="data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        video_path="videos/chunk-{episode_chunk:03d}/egocentric/episode_{episode_index:06d}.mp4",
        features=_features_meta(h, w),
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
    stats = _compute_global_stats(parquet_paths)
    with open(meta_dir / "stats_psi0.json", "w") as f:
        json.dump(stats, f, indent=2)
    with open(meta_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)
