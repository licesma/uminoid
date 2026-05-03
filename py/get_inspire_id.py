#!/usr/bin/env python3
"""
Read the HAND_ID of an Inspire RH56 dexterous hand on a given serial port.

Only the hand whose ID matches byte[2] of the request will reply, so we scan
the valid ID range [1, 254] and print whichever IDs answer.

    python get_inspire_id.py ttyUSB0
"""
import argparse
import sys

import serial

BAUDRATE = 115200
HAND_ID_ADDR_L, HAND_ID_ADDR_H = 0xE8, 0x03  # register 0x03E8


def checksum(frame: bytes) -> int:
    return sum(frame[2:-1]) & 0xFF


def probe(ser: serial.Serial, hand_id: int) -> int | None:
    frame = bytearray([0xEB, 0x90, hand_id, 0x04, 0x11, HAND_ID_ADDR_L, HAND_ID_ADDR_H, 0x01, 0x00])
    frame[-1] = checksum(frame)
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    resp = ser.read(9)
    if len(resp) < 9 or resp[0] != 0x90 or resp[1] != 0xEB:
        return None
    return resp[2]


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        usage='%(prog)s PORT [--timeout SEC]   (e.g. %(prog)s ttyUSB0)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('port', help='serial device, e.g. ttyUSB0 (or /dev/ttyUSB0)')
    ap.add_argument('--timeout', type=float, default=0.03, help='per-ID probe timeout seconds (default 0.03)')
    args = ap.parse_args()

    port = args.port if '/' in args.port else f'/dev/{args.port}'

    found = []
    with serial.Serial(port, BAUDRATE, timeout=args.timeout) as ser:
        for candidate in range(1, 255):
            got = probe(ser, candidate)
            if got is not None:
                found.append(got)

    if not found:
        sys.exit(f"No Inspire hand responded on {port} (scanned IDs 1-254).")

    unique = sorted(set(found))
    if len(unique) == 1:
        print(f"{port}: HAND_ID={unique[0]}")
    else:
        print(f"{port}: multiple IDs responded (bus has several hands?): {unique}")


if __name__ == '__main__':
    main()
