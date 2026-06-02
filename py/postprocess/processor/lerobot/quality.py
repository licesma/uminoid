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


def _dup_stats(ep_df: pd.DataFrame, cols: List[str]) -> tuple[int, float]:
    mat = ep_df[cols].to_numpy()
    if len(mat) < 2:
        return 0, 0.0
    dup_mask = np.all(mat[1:] == mat[:-1], axis=1)
    count = int(dup_mask.sum())
    ratio = count / len(dup_mask)
    return count, ratio


def compute_episode_quality(
    ep_idx: int,
    collection_id: int,
    ep_df: pd.DataFrame,
    state_cols: List[str],
    action_cols: List[str],
    n_frames_written: int,
) -> Dict[str, Any]:
    """Build the per-episode quality record (the `q` dict).

    Dup-row counts are reported for both state and action streams, but only
    the state ratio drives the `stale_async_sync` flag. Identical consecutive
    rows on the action (command) stream are normal whenever the operator
    holds still, so they would otherwise produce false positives.
    """
    dt_ms = np.diff(ep_df["host_timestamp"].to_numpy().astype(np.float64)) / 1000.0
    has_gaps = len(dt_ms) > 0

    state_dup_count, state_dup_ratio = _dup_stats(ep_df, state_cols)
    action_dup_count, action_dup_ratio = _dup_stats(ep_df, action_cols)

    flags = []
    if has_gaps and dt_ms.max() > GAP_THRESHOLD:
        flags.append("large_timestamp_gap")
    if state_dup_ratio > DUPLICATION_THRESHOLD_RATIO:
        flags.append("stale_async_sync")

    return {
        "episode_index": ep_idx,
        "collection_id": int(collection_id),
        "n_frames_csv": int(len(ep_df)),
        "n_frames_written": int(n_frames_written),
        "dt_min_ms": round(float(dt_ms.min()), 2) if has_gaps else None,
        "dt_median_ms": round(float(np.median(dt_ms)), 2) if has_gaps else None,
        "dt_p95_ms": round(float(np.quantile(dt_ms, 0.95)), 2) if has_gaps else None,
        "dt_max_ms": round(float(dt_ms.max()), 2) if has_gaps else None,
        "n_gaps_gt_100ms": int((dt_ms > 100.0).sum()) if has_gaps else 0,
        "n_gaps_gt_200ms": int((dt_ms > 200.0).sum()) if has_gaps else 0,
        "consecutive_dup_state_rows": state_dup_count,
        "consecutive_dup_state_ratio": round(state_dup_ratio, 4),
        "consecutive_dup_action_rows": action_dup_count,
        "consecutive_dup_action_ratio": round(action_dup_ratio, 4),
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
