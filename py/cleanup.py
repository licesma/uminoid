#!/usr/bin/env python3
"""Drop cancelled collections from a postprocessed session.

Reads data/<session>/{data.csv, collection_status.csv, png_frames/} and writes
data/<session>/final/{data.csv, frames/} containing only completed collections.
The frame_number column is replaced with a `frame` column of the form
`frame_000000.png` so rows can address their PNG directly.

Usage:
    python cleanup.py april_19_18:49
"""

import argparse
import shutil
from pathlib import Path

import pandas as pd

REPO_ROOT = Path(__file__).parent
FRAME_DIGITS = 6


def cleanup(session: str) -> None:
    session_dir = REPO_ROOT / "data" / session
    data_csv = session_dir / "data.csv"
    status_csv = session_dir / "collection_status.csv"
    src_frames = session_dir / "png_frames"

    final_dir = session_dir / "final"
    final_frames = final_dir / "frames"
    final_frames.mkdir(parents=True, exist_ok=True)

    data = pd.read_csv(data_csv)
    status = pd.read_csv(status_csv)

    kept_ids = set(status.loc[status["status"] == "completed", "collection_id"])
    data = data[data["collection_id"].isin(kept_ids)].copy()

    data.insert(
        data.columns.get_loc("frame_number"),
        "frame",
        data["frame_number"].apply(lambda n: f"frame_{int(n):0{FRAME_DIGITS}d}.png"),
    )
    data = data.drop(columns=["frame_number", "camera_timestamp_ms"])

    data.to_csv(final_dir / "data.csv", index=False)

    for frame_name in data["frame"]:
        shutil.copy2(src_frames / frame_name, final_frames / frame_name)

    print(
        f"Kept {len(data)} rows from {len(kept_ids)} completed collections "
        f"(dropped {len(status) - len(kept_ids)} cancelled). "
        f"Wrote {final_dir / 'data.csv'} and {len(data)} frames to {final_frames}/."
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Clean cancelled collections from a session.")
    parser.add_argument("session", help="Session folder name, e.g. april_19_18:49")
    cleanup(parser.parse_args().session)
