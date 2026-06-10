import socket
from pathlib import Path

import yaml

CONFIG_PATH = Path(__file__).resolve().parent / "config.yaml"
LOCAL_REPO = Path(__file__).resolve().parents[3]  # uminoid


def load_destinations() -> dict[str, str]:
    with open(CONFIG_PATH) as f:
        config = yaml.safe_load(f) or {}
    return config.get("destinations", {})


def local_host() -> str:
    return socket.gethostname().split(".")[0]


def is_loop(target: str) -> bool:
    host, _, path = target.partition(":")
    if host != local_host():
        return False
    return Path(path).expanduser().resolve() == LOCAL_REPO


def selectable_destinations() -> dict[str, str]:
    return {
        label: target
        for label, target in load_destinations().items()
        if not is_loop(target)
    }
