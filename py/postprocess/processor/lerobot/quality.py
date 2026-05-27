import json
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import pandas as pd

from processor.lerobot.constants import (
    DUPLICATION_THRESHOLD_RATIO,
    FRAME_RATE,
    GAP_THRESHOLD,
)


def compute_episode_quality(
    ep_idx: int,
    collection_id: int,
    ep_df: pd.DataFrame,
    joint_cols: List[str],
    n_frames_written: int,
) -> Dict[str, Any]:
    """Build the per-episode quality record (the `q` dict)."""
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
    if dt_ms.max() > GAP_THRESHOLD:
        flags.append("large_timestamp_gap")
    if dup_ratio > DUPLICATION_THRESHOLD_RATIO:
        flags.append("stale_async_sync")

    return {
        "episode_index": ep_idx,
        "collection_id": int(collection_id),
        "n_frames_csv": int(len(ep_df)),
        "n_frames_written": int(n_frames_written),
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


def write_quality_report(meta_dir: Path, quality_records: List[Dict[str, Any]]) -> Path:
    """Write `meta/quality_report.json`. Returns the report path."""
    flagged = [q for q in quality_records if q["flags"]]
    report = {
        "fps_assumed": FRAME_RATE,
        "expected_dt_ms": 1000.0 / FRAME_RATE,  # ~33.3 ms at 30Hz
        "flag_dt_ms": GAP_THRESHOLD,
        "flag_dup_ratio": DUPLICATION_THRESHOLD_RATIO,
        "n_episodes_total": len(quality_records),
        "n_episodes_flagged": len(flagged),
        "flagged_episode_indices": [q["episode_index"] for q in flagged],
        "flagged_collection_ids": [q["collection_id"] for q in flagged],
        "per_episode": quality_records,
    }
    report_path = meta_dir / "quality_report.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    return report_path
