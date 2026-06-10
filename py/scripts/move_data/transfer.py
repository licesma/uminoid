import subprocess
from pathlib import Path


def rsync_folder(local_folder: Path, target: str, kind: str) -> int:

    remote_dest = f"{target}/{kind}/"
    cmd = ["rsync", "-avz", "--", str(local_folder), remote_dest]
    print(f"\nRunning: {' '.join(cmd)}\n")
    return subprocess.run(cmd).returncode
