#!/usr/bin/env python3
"""
Sweep the index finger of an Inspire RH56 hand directly over serial.

Bypasses DDS / the inspire_g1 bridge — useful for confirming a hand responds
on a given port and HAND_ID. Everything is hardcoded; just run it.

    python move_inspire_index.py
"""
import signal
import sys
import time

import serial

PORT = "/dev/ttyUSB0"
HAND_ID = 1
FINGER = "index"
BAUDRATE = 115200
STEP = 0.05
PERIOD_S = 0.1

N_FINGERS = 6
FINGER_NAMES = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation")
FINGER_INDEX = FINGER_NAMES.index(FINGER)

_REG_SET_POS_L, _REG_SET_POS_H = 0xCE, 0x05      # ANGLE_SET (1486)
_REG_GET_POS_L, _REG_GET_POS_H = 0x0A, 0x06      # ANGLE_ACT (1546)
_REG_ERROR_L,   _REG_ERROR_H   = 0x46, 0x06      # ERROR(0)  (1606), 6 bytes
_REG_STATUS_L,  _REG_STATUS_H  = 0x4C, 0x06      # STATUS(0) (1612), 6 bytes
_REG_TEMP_L,    _REG_TEMP_H    = 0x52, 0x06      # TEMP(0)   (1618), 6 bytes
_REG_CLEAR_ERR_L, _REG_CLEAR_ERR_H = 0xEC, 0x03  # CLEAR_ERROR (1004)

STATUS_NAMES = {0: "unclench", 1: "grasping", 2: "at-target", 3: "force-stop",
                5: "current-protect", 6: "locked-rotor", 7: "actuator-fault"}

ERROR_BITS = {
    0x01: "locked-rotor",
    0x02: "over-temp",
    0x04: "over-current",
    0x08: "motor-abnormal",
    0x10: "comm-error",
}


def checksum(frame: bytes) -> int:
    return sum(frame[2:-1]) & 0xFF


def build_set_position(hand_id: int, q_units: list[int]) -> bytes:
    frame = bytearray(20)
    frame[0:2] = b"\xEB\x90"
    frame[2] = hand_id & 0xFF
    frame[3] = 0x0F
    frame[4] = 0x12  # write
    frame[5] = _REG_SET_POS_L
    frame[6] = _REG_SET_POS_H
    for i, v in enumerate(q_units):
        v &= 0xFFFF
        frame[7 + 2 * i] = v & 0xFF
        frame[8 + 2 * i] = (v >> 8) & 0xFF
    frame[19] = checksum(bytes(frame))
    return bytes(frame)


def build_get_position(hand_id: int) -> bytes:
    frame = bytearray([0xEB, 0x90, hand_id & 0xFF, 0x04, 0x11,
                       _REG_GET_POS_L, _REG_GET_POS_H, 0x0C, 0x00])
    frame[-1] = checksum(bytes(frame))
    return bytes(frame)


def parse_get_position(resp: bytes) -> list[float] | None:
    if len(resp) != 20 or checksum(resp) != resp[-1]:
        return None
    out = []
    for i in range(N_FINGERS):
        lo = resp[7 + 2 * i]
        hi = resp[8 + 2 * i]
        out.append(((hi << 8) | lo) / 1000.0)
    return out


def write_positions(ser: serial.Serial, q: list[float]) -> None:
    units = [max(0, min(1000, int(round(v * 1000)))) for v in q]
    ser.reset_input_buffer()
    ser.write(build_set_position(HAND_ID, units))
    ser.flush()
    ser.read(9)


def read_positions(ser: serial.Serial) -> list[float] | None:
    ser.reset_input_buffer()
    ser.write(build_get_position(HAND_ID))
    ser.flush()
    return parse_get_position(ser.read(20))


def read_errors(ser: serial.Serial) -> list[int] | None:
    return _read_byte_array(ser, _REG_ERROR_L, _REG_ERROR_H)


def read_status(ser: serial.Serial) -> list[int] | None:
    return _read_byte_array(ser, _REG_STATUS_L, _REG_STATUS_H)


def read_temp(ser: serial.Serial) -> list[int] | None:
    return _read_byte_array(ser, _REG_TEMP_L, _REG_TEMP_H)


def _read_byte_array(ser: serial.Serial, addr_l: int, addr_h: int, n: int = 6) -> list[int] | None:
    """Read N bytes starting at register addr. Response is N+8 bytes."""
    frame = bytearray([0xEB, 0x90, HAND_ID & 0xFF, 0x04, 0x11,
                       addr_l, addr_h, n & 0xFF, 0x00])
    frame[-1] = checksum(bytes(frame))
    ser.reset_input_buffer()
    ser.write(bytes(frame))
    ser.flush()
    resp = ser.read(n + 8)
    if len(resp) != n + 8 or checksum(resp) != resp[-1]:
        return None
    return list(resp[7:7 + n])


def clear_errors(ser: serial.Serial) -> None:
    frame = bytearray([0xEB, 0x90, HAND_ID & 0xFF, 0x04, 0x12,
                       _REG_CLEAR_ERR_L, _REG_CLEAR_ERR_H, 0x01, 0x00])
    frame[-1] = checksum(bytes(frame))
    ser.reset_input_buffer()
    ser.write(bytes(frame))
    ser.flush()
    ser.read(9)


def describe_errors(errs: list[int]) -> str:
    parts = []
    for i, e in enumerate(errs):
        if e == 0:
            continue
        flags = [name for bit, name in ERROR_BITS.items() if e & bit]
        parts.append(f"{FINGER_NAMES[i]}=0x{e:02X}({'|'.join(flags) or '?'})")
    return ", ".join(parts) if parts else "no errors"


def main() -> None:
    running = [True]
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))

    with serial.Serial(PORT, BAUDRATE, timeout=0.1) as ser:
        probe = read_positions(ser)
        if probe is None:
            sys.exit(f"No response on {PORT} at id={HAND_ID}.")
        print(f"Connected to {PORT} id={HAND_ID}. Initial state: "
              + ", ".join(f"{v:.3f}" for v in probe))

        errs = read_errors(ser)
        if errs is None:
            print("WARN: could not read ERROR registers")
        else:
            print(f"ERROR : {describe_errors(errs)}")
            if any(errs):
                print("Attempting CLEAR_ERROR...")
                clear_errors(ser)
                time.sleep(0.2)
                errs2 = read_errors(ser)
                print(f"ERROR : {describe_errors(errs2 or [])} (after clear)")
                if errs2 and any(errs2):
                    print("Errors remain (over-temp is not clearable until cooled).")

        st = read_status(ser)
        if st is not None:
            print("STATUS: " + ", ".join(
                f"{FINGER_NAMES[i]}={STATUS_NAMES.get(v, '?')}({v})" for i, v in enumerate(st)))
        tp = read_temp(ser)
        if tp is not None:
            print(f"TEMP C: {tp}  (zeros => actuator MCU not reachable)")

        print(f"\nSweeping {FINGER} (open <-> closed). Ctrl+C to stop.\n")

        q = [1.0] * N_FINGERS
        target = 1.0
        closing = True
        while running[0]:
            if closing:
                target -= STEP
                if target <= 0.0:
                    target = 0.0
                    closing = False
            else:
                target += STEP
                if target >= 1.0:
                    target = 1.0
                    closing = True

            q[FINGER_INDEX] = target
            write_positions(ser, q)

            actual = read_positions(ser)
            actual_v = actual[FINGER_INDEX] if actual else float("nan")
            print(f"target: {target:.2f}  actual: {actual_v:.3f}        \r",
                  end="", flush=True)

            time.sleep(PERIOD_S)

        q[FINGER_INDEX] = 1.0
        write_positions(ser, q)
        print(f"\nDone. {FINGER} returned to open.")


if __name__ == "__main__":
    main()
