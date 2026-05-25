from pathlib import Path

import pandas as pd

from helpers.error_check import ensure

STATUS_FILE = "collection_status.csv"


def load_completed_collections(episode_path: Path) -> pd.DataFrame:
    path = episode_path / STATUS_FILE
    ensure(path.exists(), f"{path} does not exist")
    df = pd.read_csv(path)
    return df[df["status"] == "completed"].reset_index(drop=True)


def filter_completed(df: pd.DataFrame, status_df: pd.DataFrame) -> pd.DataFrame:
    """Keep only rows whose collection_id is in the completed set."""
    completed = set(status_df["collection_id"].astype(int))
    return df[df["collection_id"].isin(completed)].reset_index(drop=True)
