#!/usr/bin/env python3
"""Package a postprocessed session into a sibling ``final_data/<session>/``.

Bundles two pieces of an already-processed session:

    data/<session>/data.csv         -> final_data/<session>/data.csv
    data/<session>/png_frames/*.png -> final_data/<session>/frames/*.png

That layout matches what the LeRobot-style converters expect (one ``data.csv``
plus a sibling ``frames/`` directory of PNGs).  Source files stay in place.

Usage:
    python build_final_dataset.py may_06_12:35
"""

import argparse
import shutil
from pathlib import Path

import pandas as pd

REPO_ROOT = Path(__file__).parent.parent
DATA_DIR       = REPO_ROOT / "data"
FINAL_DATA_DIR = REPO_ROOT / "final_data"


def build(session: str, overwrite: bool = False) -> Path:
    src = DATA_DIR / session
    if not src.is_dir():
        raise FileNotFoundError(f"Session source directory not found: {src}")

    csv_src = src / "data.csv"
    if not csv_src.is_file():
        raise FileNotFoundError(
            f"Missing {csv_src}. Run `python py/postprocess_data.py {session}` first."
        )

    pngs_src = src / "png_frames"
    if not pngs_src.is_dir():
        raise FileNotFoundError(
            f"Missing {pngs_src}. Run `python py/compress_raw_frames.py "
            f"{src / 'frames'}` first."
        )

    dst = FINAL_DATA_DIR / session
    if dst.exists():
        if not overwrite:
            raise FileExistsError(f"{dst} already exists (use --overwrite to replace)")
        shutil.rmtree(dst)
    dst.mkdir(parents=True)

    shutil.copy2(csv_src, dst / "data.csv")

    # Copy only the PNG frames referenced by data.csv, to skip any frames whose
    # collection_id was dropped by postprocess (e.g. cancelled episodes).
    referenced = pd.read_csv(csv_src, usecols=["frame"])["frame"].astype(str).unique()
    frames_dst = dst / "frames"
    frames_dst.mkdir()
    n_copied = 0
    n_missing = 0
    for fname in referenced:
        src_png = pngs_src / fname
        if src_png.is_file():
            shutil.copy2(src_png, frames_dst / fname)
            n_copied += 1
        else:
            n_missing += 1
    msg = f"Packaged {session}: data.csv + {n_copied} PNG frame(s) -> {dst}"
    if n_missing:
        msg += f"  ({n_missing} referenced PNG(s) missing from {pngs_src})"
    print(msg)
    return dst


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Package a postprocessed session.")
    parser.add_argument("session", help="Session folder name, e.g. may_06_12:35")
    parser.add_argument("--overwrite", action="store_true",
                        help="Replace an existing final_data/<session>/ directory.")
    args = parser.parse_args()
    build(args.session, overwrite=args.overwrite)
