"""
Load the raw per-stream CSVs for a recorded session.

Returns a dict keyed by stream name; only streams present on disk are
included. Recognized streams:

    camera, left_arm, right_arm, dex3_hand, inspire_hand

For arm sides, ``{side}_measured.csv`` is preferred over ``{side}_arm.csv``
when both exist (the measured CSV is already in robot-frame radians; the
arm CSV is raw dynamixel bits).

Usage:
    from load_session import load_session

    streams = load_session(
        "may_06_12:35",
        start_time=100_000,   # microseconds (host_timestamp)
        end_time=500_000,
    )
    camera = streams["camera"]
    right_arm = streams.get("right_arm")  # may be None

    # Or run directly:
    python load_session.py may_06_12:35 --start 100000 --end 500000
"""

import argparse
import os
from typing import Dict, Optional

import pandas as pd

REPO_ROOT = os.path.dirname(os.path.dirname(__file__))
DATA_DIR = os.path.join(REPO_ROOT, "data")

# (stream_name, [candidate_filenames])  — first existing file wins
STREAM_CANDIDATES = [
    ("camera",       ["camera.csv"]),
    ("left_arm",     ["left_measured.csv",  "left_arm.csv"]),
    ("right_arm",    ["right_measured.csv", "right_arm.csv"]),
    ("dex3_hand",    ["dex3_hand.csv"]),
    ("inspire_hand", ["inspire_hand.csv"]),
]


def load_session(
    session: str,
    start_time: Optional[float] = None,
    end_time: Optional[float] = None,
) -> Dict[str, pd.DataFrame]:
    """Load all per-stream CSVs that exist on disk for ``session``.

    Parameters
    ----------
    session : str
        Session directory name under data/ (e.g. "may_06_12:35").
    start_time, end_time : float, optional
        Inclusive bounds on host_timestamp (microseconds). Unbounded if None.

    Returns
    -------
    dict[str, pd.DataFrame]
        One entry per stream that was found on disk. All DataFrames share the
        ``host_timestamp`` column (microseconds from steady_clock origin).
    """
    session_dir = os.path.join(DATA_DIR, session)
    if not os.path.isdir(session_dir):
        raise FileNotFoundError(f"Session directory not found: {session_dir}")

    def _filter(df: pd.DataFrame) -> pd.DataFrame:
        if "host_timestamp" not in df.columns or (start_time is None and end_time is None):
            return df.reset_index(drop=True)
        mask = pd.Series(True, index=df.index)
        if start_time is not None:
            mask &= df["host_timestamp"] >= start_time
        if end_time is not None:
            mask &= df["host_timestamp"] <= end_time
        return df[mask].reset_index(drop=True)

    streams: Dict[str, pd.DataFrame] = {}
    for name, candidates in STREAM_CANDIDATES:
        for fname in candidates:
            path = os.path.join(session_dir, fname)
            if os.path.exists(path):
                streams[name] = _filter(pd.read_csv(path))
                break
    return streams


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load a session's raw per-stream CSVs.")
    parser.add_argument("session", help="Session name under data/ (e.g. may_06_12:35)")
    parser.add_argument("--start", type=float, default=None, metavar="US",
                        help="Start host_timestamp in microseconds (inclusive)")
    parser.add_argument("--end", type=float, default=None, metavar="US",
                        help="End host_timestamp in microseconds (inclusive)")
    args = parser.parse_args()

    streams = load_session(args.session, start_time=args.start, end_time=args.end)
    for name, df in streams.items():
        print(f"\n=== {name} ({len(df)} rows, {len(df.columns)} cols) ===")
        print(df.head())
