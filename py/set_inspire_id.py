#!/usr/bin/env python3
"""
Assign a HAND_ID to an Inspire RH56 dexterous hand (one-off setup).

Plug in ONE hand at a time, then run for example:

    python set_inspire_id.py ttyUSB0 --new-id 2

Convention used by collect.cpp:
    left_hand  -> HAND_ID = 1  (factory default, no reassignment needed)
    right_hand -> HAND_ID = 2

The new ID is persisted to flash via the SAVE register.
"""
import argparse
import sys
import time

import serial

BAUDRATE = 115200
HAND_ID_ADDR_L, HAND_ID_ADDR_H = 0xE8, 0x03  # register 0x03E8
SAVE_ADDR_L,    SAVE_ADDR_H    = 0xED, 0x03  # register 0x03ED


def checksum(frame: bytes) -> int:
    # Sum bytes[2 .. len-2] (skip 0xEB 0x90 header and the checksum slot).
    return sum(frame[2:-1]) & 0xFF


def write_byte(ser: serial.Serial, hand_id: int, addr_l: int, addr_h: int, value: int) -> bytes:
    frame = bytearray([0xEB, 0x90, hand_id, 0x04, 0x12, addr_l, addr_h, value, 0x00])
    frame[-1] = checksum(frame)
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    return ser.read(9)


def read_byte(ser: serial.Serial, hand_id: int, addr_l: int, addr_h: int) -> int | None:
    frame = bytearray([0xEB, 0x90, hand_id, 0x04, 0x11, addr_l, addr_h, 0x01, 0x00])
    frame[-1] = checksum(frame)
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    resp = ser.read(9)
    if len(resp) < 9 or resp[0] != 0x90 or resp[1] != 0xEB:
        return None
    # Response layout: 90 EB <id> 04 11 <addr_l> <addr_h> <data> <ck>
    return resp[7]


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        usage='%(prog)s PORT --new-id N [--current-id N]   (e.g. %(prog)s ttyUSB0 --new-id 2)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('port', help='serial device, e.g. ttyUSB0 (or /dev/ttyUSB0)')
    ap.add_argument('--new-id', type=int, required=True, help='new HAND_ID (1-254)')
    ap.add_argument('--current-id', type=int, default=1, help='current HAND_ID (default 1 = factory)')
    args = ap.parse_args()

    if not 1 <= args.new_id <= 254:
        sys.exit(f"--new-id must be in [1,254], got {args.new_id}")
    if not 1 <= args.current_id <= 254:
        sys.exit(f"--current-id must be in [1,254], got {args.current_id}")

    port = args.port if '/' in args.port else f'/dev/{args.port}'

    with serial.Serial(port, BAUDRATE, timeout=0.1) as ser:
        # 1) Confirm the hand is reachable at the claimed current-id.
        got = read_byte(ser, args.current_id, HAND_ID_ADDR_L, HAND_ID_ADDR_H)
        if got is None:
            sys.exit(
                f"No response on {port} at id={args.current_id}. "
                "Check wiring, or pass --current-id <actual-id>."
            )
        if got != args.current_id:
            sys.exit(f"Sanity check failed: HAND_ID register reports {got}, expected {args.current_id}.")
        print(f"Found hand on {port} with HAND_ID={args.current_id}")

        if args.new_id == args.current_id:
            print("New ID equals current ID — nothing to do.")
            return

        # 2) Write the new ID. After this, the hand answers to new_id.
        print(f"Setting HAND_ID: {args.current_id} -> {args.new_id}")
        write_byte(ser, args.current_id, HAND_ID_ADDR_L, HAND_ID_ADDR_H, args.new_id)

        # 3) Persist to flash (SAVE register = 1).
        print("Saving to flash...")
        write_byte(ser, args.new_id, SAVE_ADDR_L, SAVE_ADDR_H, 0x01)

        # 4) Verify — the hand is unresponsive while it writes flash, so retry.
        got = None
        for _ in range(20):
            time.sleep(0.1)
            got = read_byte(ser, args.new_id, HAND_ID_ADDR_L, HAND_ID_ADDR_H)
            if got == args.new_id:
                break
        if got != args.new_id:
            sys.exit(f"Verification failed: HAND_ID register reports {got}, expected {args.new_id}.")
        print(f"OK. {port} is now HAND_ID={args.new_id} (persisted).")


if __name__ == '__main__':
    main()
