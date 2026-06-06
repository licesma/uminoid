import sys
from pathlib import Path

PY_DIR = Path(__file__).resolve().parents[2]  # uminoid/py
sys.path.insert(0, str(PY_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from paths import DATA_DIR, TRAINING_DATA_DIR

import prompts
from config_loader import load_host_repos
from ssh_hosts import reachable_destinations
from transfer import rsync_folder

LOCAL_DIRS = {
    "data": DATA_DIR,
    "training_data": TRAINING_DATA_DIR,
}


def list_folders(directory: Path) -> list[str]:
    return sorted(p.name for p in directory.iterdir() if p.is_dir())


def main() -> int:
    kind = prompts.select_data_kind()

    source_dir = LOCAL_DIRS[kind]
    folders = list_folders(source_dir)
    if not folders:
        print(f"No folders found under {source_dir}", file=sys.stderr)
        return 1
    folder = prompts.select_folder(kind, folders)

    host_repos = load_host_repos()
    destinations = reachable_destinations(list(host_repos))
    if not destinations:
        print("No reachable destination hosts found in ~/.ssh/config", file=sys.stderr)
        return 1
    host = prompts.select_destination(destinations)

    return rsync_folder(source_dir / folder, host, host_repos[host], kind)


if __name__ == "__main__":
    raise SystemExit(main())
