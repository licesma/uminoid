#!/usr/bin/env python3
"""
Create a persistent /dev/<arm_name> symlink for a Dynamixel U2D2 adapter by
writing a udev rule matched on the FTDI chip's per-unit serial number.

Must be run as root (sudo).

    sudo python setup_arm_udev.py left_arm FT94EJO0
    sudo python setup_arm_udev.py right_arm FT94EJNU
"""
import argparse
import os
import subprocess
import sys
import time

RULES_FILE = "/etc/udev/rules.d/99-uminoid-arms.rules"
VENDOR_ID  = "0403"  # FTDI


def make_rule(arm_name: str, serial: str) -> str:
    return (f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{VENDOR_ID}", '
            f'ATTRS{{serial}}=="{serial}", SYMLINK+="{arm_name}", '
            f'RUN+="/bin/sh -c \'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer\'"')


def load_rules() -> dict[str, str]:
    """Return {arm_name: serial} for rules already in the file."""
    rules: dict[str, str] = {}
    if not os.path.exists(RULES_FILE):
        return rules
    with open(RULES_FILE) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # Parse SYMLINK+="<name>" and ATTRS{serial}=="<serial>"
            name = serial = None
            for token in line.split(","):
                token = token.strip()
                if token.startswith('SYMLINK+='):
                    name = token.split('"')[1]
                elif 'serial' in token and 'ATTRS' in token:
                    serial = token.split('"')[1]
            if name and serial:
                rules[name] = serial
    return rules


def write_rules(rules: dict[str, str]) -> None:
    header = (
        "# Stable /dev symlinks for the Dynamixel U2D2 adapters.\n"
        "# Managed by setup_arm_udev.py — matched by FTDI per-unit serial number.\n"
    )
    lines = [make_rule(name, serial) for name, serial in sorted(rules.items())]
    with open(RULES_FILE, "w") as f:
        f.write(header + "\n".join(lines) + "\n")


def reload_udev() -> None:
    subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
    subprocess.run(["udevadm", "trigger"], check=True)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        usage="sudo python %(prog)s ARM_NAME SERIAL_ID   (e.g. %(prog)s left_arm FT94EJO0)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("arm_name", help="symlink name, e.g. left_arm or right_arm")
    ap.add_argument("serial_id", help="FTDI serial number, e.g. FT94EJO0")
    args = ap.parse_args()

    if os.geteuid() != 0:
        sys.exit("Error: must be run as root — use sudo.")

    rules = load_rules()

    prev = rules.get(args.arm_name)
    if prev == args.serial_id:
        print(f"Rule already up to date: /dev/{args.arm_name} -> {args.serial_id}")
    else:
        if prev:
            print(f"Updating /dev/{args.arm_name}: {prev} -> {args.serial_id}")
        else:
            print(f"Adding rule: /dev/{args.arm_name} -> {args.serial_id}")
        rules[args.arm_name] = args.serial_id
        write_rules(rules)

    print("Reloading udev rules...")
    reload_udev()
    time.sleep(0.5)

    dev_path = f"/dev/{args.arm_name}"
    if os.path.exists(dev_path):
        real = os.path.realpath(dev_path)
        print(f"OK. {dev_path} -> {real}")
    else:
        sys.exit(f"Error: {dev_path} did not appear after udev reload — check wiring.")


if __name__ == "__main__":
    main()
