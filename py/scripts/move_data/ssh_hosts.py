from pathlib import Path

SSH_CONFIG_PATH = Path.home() / ".ssh" / "config"


def ssh_host_aliases() -> set[str]:
    """Every alias declared with a `Host` line in ~/.ssh/config."""
    if not SSH_CONFIG_PATH.exists():
        return set()

    aliases: set[str] = set()
    for line in SSH_CONFIG_PATH.read_text().splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        key, _, value = stripped.partition(" ")
        if key.lower() == "host":
            aliases.update(token for token in value.split() if "*" not in token)
    return aliases


def reachable_destinations(known_hosts: list[str]) -> list[str]:
    """Known machines we can SSH into.

    The machine running this script has no SSH entry pointing at itself, so it
    is excluded automatically — leaving only the other machines as destinations.
    """
    aliases = ssh_host_aliases()
    return [host for host in known_hosts if host in aliases]
