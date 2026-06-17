"""Entry point: render a task config to CLI args and exec upstream Psi-0's train.py via torchrun.
"""

from __future__ import annotations

import os
import resource
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # repo root, so `py` package is importable when run directly

from InquirerPy import inquirer

from py.paths import ROOT_DIR
from py.train.tasks import TASKS


PSI0_TRAIN_SCRIPT = ROOT_DIR / "third_party" / "Psi0" / "scripts" / "train.py"
VENV_TORCHRUN = ROOT_DIR / ".venv-psi" / "bin" / "torchrun"


def _load_dotenv_if_present() -> None:
    """Populate os.environ from .env at repo root (mirrors `set -a; source .env; set +a`)."""
    env_path = ROOT_DIR / ".env"
    if not env_path.is_file():
        return
    for line in env_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        # strip optional surrounding quotes
        value = value.strip().strip('"').strip("'")
        os.environ.setdefault(key.strip(), value)


def _raise_nofile_limit(target: int = 65535) -> None:
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    if soft >= target:
        return
    new_soft = min(target, hard)
    resource.setrlimit(resource.RLIMIT_NOFILE, (new_soft, hard))


def _select_task() -> str:
    """Interactively pick a task. The task fixes the data path and all other flags."""
    return inquirer.select(
        message="Select training task:",
        choices=sorted(TASKS),
    ).execute()


def _input_experiment(default: str) -> str:
    """Prompt for the experiment (run/output) name, pre-filled with the task's default."""
    return inquirer.text(
        message="Experiment name:",
        default=default,
    ).execute()


def main() -> int:
    _load_dotenv_if_present()
    _raise_nofile_limit()

    if not PSI0_TRAIN_SCRIPT.is_file():
        raise FileNotFoundError(f"Psi-0 train.py not found at {PSI0_TRAIN_SCRIPT}")

    torchrun = str(VENV_TORCHRUN) if VENV_TORCHRUN.is_file() else "torchrun"

    task_name = _select_task()
    cfg = TASKS[task_name]()
    cfg.experiment = _input_experiment(cfg.experiment)

    cmd = [
        torchrun,
        "--nproc_per_node=1",
        "--master_port=29500",
        str(PSI0_TRAIN_SCRIPT),
    ] + cfg.to_cli_args()

    print(f"Training on GPU {os.environ.get('CUDA_VISIBLE_DEVICES', '<unset>')}")
    print(f"Experiment: {cfg.experiment}")
    print(f"Command:\n  {' '.join(cmd)}\n")

    return subprocess.run(cmd, cwd=ROOT_DIR).returncode


if __name__ == "__main__":
    sys.exit(main())
