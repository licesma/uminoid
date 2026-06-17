import sys
from pathlib import Path

PY_DIR = Path(__file__).resolve().parents[2]  # uminoid/py
sys.path.insert(0, str(PY_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from paths import DATA_DIR, ROOT_DIR, RUNS_DIR, TRAINING_DATA_DIR

import prompts
from config_loader import selectable_destinations
from transfer import rsync_folder

LOCAL_DIRS = {
    "data": DATA_DIR,
    "training_data": TRAINING_DATA_DIR,
    ".runs": RUNS_DIR / "finetune",
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

    destinations = selectable_destinations()
    if not destinations:
        print("No destinations configured in config.yaml", file=sys.stderr)
        return 1
    label = prompts.select_destination(list(destinations))

    local_folder = source_dir / folder
    remote_subdir = local_folder.parent.relative_to(ROOT_DIR)
    return rsync_folder(local_folder, destinations[label], remote_subdir)


if __name__ == "__main__":
    raise SystemExit(main())
