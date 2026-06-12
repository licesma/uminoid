import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # add py/ to sys.path

from InquirerPy import inquirer

from paths import DATA_DIR


def select_data_sources() -> list[str]:
    folders = sorted(p.name for p in DATA_DIR.iterdir() if p.is_dir())
    if not folders:
        print(f"No data folders found under {DATA_DIR}", file=sys.stderr)
        return []

    selected = inquirer.fuzzy(
        message="Select data sources (tab to toggle, enter to confirm):",
        choices=folders,
        multiselect=True,
        max_height="70%",
    ).execute()

    return selected or []


def main() -> int:
    selected = select_data_sources()
    print("\nSelected data sources:")
    for name in selected:
        print(f"  - {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
