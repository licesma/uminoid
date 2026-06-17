from pathlib import Path

import yaml

CONFIG_PATH = Path(__file__).resolve().parent / "config.yaml"


def load_host_repos() -> dict[str, str]:
    """Map every known machine name to its uminoid repo path."""
    with open(CONFIG_PATH) as f:
        config = yaml.safe_load(f) or {}
    return config.get("hosts", {})


def selectable_destinations() -> dict[str, str]:
    """Map every destination label to its rsync target."""
    with open(CONFIG_PATH) as f:
        config = yaml.safe_load(f) or {}
    return config.get("destinations", {})
