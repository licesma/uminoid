from pathlib import Path

import pandas as pd

from helpers.error_check import ensure
from processor.components.status import filter_completed

CAMERA_FILE = "camera.csv"


def load_camera(episode_path: Path, status_df: pd.DataFrame) -> pd.DataFrame:
    path = episode_path / CAMERA_FILE
    ensure(path.exists(), f"{path} does not exist")
    return filter_completed(pd.read_csv(path), status_df)
