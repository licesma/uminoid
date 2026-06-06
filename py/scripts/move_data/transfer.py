import subprocess
from pathlib import Path


def rsync_folder(local_folder: Path, host: str, remote_repo: str, kind: str) -> int:
    """Copy a local data folder to <remote_repo>/<kind>/ on the destination host.

    No trailing slash on the source, so the folder is recreated by name under
    the destination; the remote `~` is expanded by the destination shell.
    """
    remote_dest = f"{host}:{remote_repo}/{kind}/"
    cmd = ["rsync", "-avz", "--", str(local_folder), remote_dest]
    print(f"\nRunning: {' '.join(cmd)}\n")
    return subprocess.run(cmd).returncode
