"""
Load session data into 4 DataFrames with optional time filtering.

Usage:
    from load_session import load_session

    left_arm, right_arm, inspire_hand, camera = load_session(
        "april_18_15:24",
        start_time=100_000,   # microseconds (host_timestamp)
        end_time=500_000,
    )

    # Or run directly:
    python load_session.py april_18_15:24 --start 100000 --end 500000
"""

import argparse
import os
import pandas as pd

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


def load_session(
    session: str,
    start_time: float | None = None,
    end_time: float | None = None,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """
    Load a recorded session into 4 DataFrames.

    Parameters
    ----------
    session : str
        Session directory name under data/ (e.g. "april_18_15:24").
    start_time : float, optional
        Lower bound for host_timestamp (microseconds, inclusive). No lower bound if None.
    end_time : float, optional
        Upper bound for host_timestamp (microseconds, inclusive). No upper bound if None.

    Returns
    -------
    left_arm, right_arm, inspire_hand, camera : pd.DataFrame
        All DataFrames share a common host_timestamp column (microseconds from steady_clock origin).
    """
    session_dir = os.path.join(DATA_DIR, session)
    if not os.path.isdir(session_dir):
        raise FileNotFoundError(f"Session directory not found: {session_dir}")

    left_arm     = pd.read_csv(os.path.join(session_dir, "left_arm.csv"))
    right_arm    = pd.read_csv(os.path.join(session_dir, "right_arm.csv"))
    inspire_hand = pd.read_csv(os.path.join(session_dir, "inspire_hand.csv"))
    camera       = pd.read_csv(os.path.join(session_dir, "camera.csv"))

    def _filter(df: pd.DataFrame) -> pd.DataFrame:
        mask = pd.Series(True, index=df.index)
        if start_time is not None:
            mask &= df["host_timestamp"] >= start_time
        if end_time is not None:
            mask &= df["host_timestamp"] <= end_time
        return df[mask].reset_index(drop=True)

    return _filter(left_arm), _filter(right_arm), _filter(inspire_hand), _filter(camera)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load a session into DataFrames.")
    parser.add_argument("session", help="Session name under data/ (e.g. april_18_15:24)")
    parser.add_argument("--start", type=float, default=None, metavar="US",
                        help="Start host_timestamp in microseconds (inclusive)")
    parser.add_argument("--end", type=float, default=None, metavar="US",
                        help="End host_timestamp in microseconds (inclusive)")
    args = parser.parse_args()

    left_arm, right_arm, inspire_hand, camera = load_session(
        args.session,
        start_time=args.start,
        end_time=args.end,
    )

    print(f"\n=== left_arm ({len(left_arm)} rows) ===")
    print(left_arm.head())

    print(f"\n=== right_arm ({len(right_arm)} rows) ===")
    print(right_arm.head())

    print(f"\n=== inspire_hand ({len(inspire_hand)} rows) ===")
    print(inspire_hand.head())

    print(f"\n=== camera ({len(camera)} rows) ===")
    print(camera.head())
