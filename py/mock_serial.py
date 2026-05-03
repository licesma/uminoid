"""
Fake serial device that streams binary frames over a PTY.

Frame layout (little-endian, 40 bytes total):
  uint16  header        0xAA55 sync word
  uint64  timestamp_us  microseconds since an arbitrary epoch
  uint16  values[14]    encoder readings (0–4095)
  uint16  crc           CRC-16/CCITT over header..values

Keyboard control (hold to change continuously):
  up / down
  1  /  q  -> encoder 0
  2  /  w  -> encoder 1
  3  /  e  -> encoder 2
  4  /  r  -> encoder 3
  5  /  t  -> encoder 4
  6  /  y  -> encoder 5
  7  /  u  -> encoder 6
  a  /  z  -> encoder 7
  s  /  x  -> encoder 8
  d  /  c  -> encoder 9
  f  /  v  -> encoder 10
  g  /  b  -> encoder 11
  h  /  n  -> encoder 12
  j  /  m  -> encoder 13

Values wrap in [0, 4095].
"""

import os, pty, time, sys, termios, tty, select, fcntl, errno, struct

from joints import joint_name

HZ = 500
DT = 1.0 / HZ
N = 14
RATE = 1000        # units per second while a key is held
STEP = RATE * DT   # applied every tick per held key
DISPLAY_EVERY = 25 # refresh display every N ticks (~20 Hz)
INITIAL_TIME_US = 2003006800000

FRAME_HEADER = 0xAA55
FRAME_FMT = "<HQ14H"                        # header(u16) + ts(u64) + 14×value(u16)
FRAME_PAYLOAD_SIZE = struct.calcsize(FRAME_FMT)  # 38 bytes
FRAME_SIZE = FRAME_PAYLOAD_SIZE + 2              # +2 for CRC-16

# How long (seconds) after the last key event before we consider it released.
# Must be longer than the OS key-repeat interval (~30 ms) but short enough
# that release feels instant (~150 ms works well).
HOLD_TIMEOUT = 0.15

COL_W = 8  # characters per encoder column

_SHORT_NAMES = {
    "left_shoulder_pitch":  "L_sh_pit",
    "left_shoulder_roll":   "L_sh_rol",
    "left_shoulder_yaw":    "L_sh_yaw",
    "left_elbow":           "L_elbow",
    "left_wrist_roll":      "L_wr_rol",
    "left_wrist_pitch":     "L_wr_pit",
    "left_wrist_yaw":       "L_wr_yaw",
    "right_shoulder_pitch": "R_sh_pit",
    "right_shoulder_roll":  "R_sh_rol",
    "right_shoulder_yaw":   "R_sh_yaw",
    "right_elbow":          "R_elbow",
    "right_wrist_roll":     "R_wr_rol",
    "right_wrist_pitch":    "R_wr_pit",
    "right_wrist_yaw":      "R_wr_yaw",
}

def short_name(full_name: str) -> str:
    return _SHORT_NAMES[full_name]

KEYMAP = {
    # enc 0-6: numbers row (up) / qwerty row (down)
    "1": (0,  +1), "q": (0,  -1),
    "2": (1,  +1), "w": (1,  -1),
    "3": (2,  +1), "e": (2,  -1),
    "4": (3,  +1), "r": (3,  -1),
    "5": (4,  +1), "t": (4,  -1),
    "6": (5,  +1), "y": (5,  -1),
    "7": (6,  +1), "u": (6,  -1),
    # enc 7-13: home row (up) / bottom row (down)
    "a": (7,  +1), "z": (7,  -1),
    "s": (8,  +1), "x": (8,  -1),
    "d": (9,  +1), "c": (9,  -1),
    "f": (10, +1), "v": (10, -1),
    "g": (11, +1), "b": (11, -1),
    "h": (12, +1), "n": (12, -1),
    "j": (13, +1), "m": (13, -1),
}

def wrap(x, lo=0.0, hi=4095.0):
    r = hi - lo + 1
    return ((x - lo) % r) + lo


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def pack_frame(t_us: int, vals: list[float]) -> bytes:
    payload = struct.pack(FRAME_FMT, FRAME_HEADER, t_us, *(int(v) for v in vals))
    crc = crc16_ccitt(payload[2:])  # CRC covers timestamp + values, not header
    return payload + struct.pack("<H", crc)


def fmt_header(indices):
    return " | ".join(f"{short_name(joint_name(i)):>{COL_W}}" for i in indices)

def fmt_values(vals):
    return " | ".join(f"{int(v):04d}".rjust(COL_W) for v in vals)

def read_keys_nonblocking(fd: int) -> list[str]:
    """Return all currently available single-char keypresses (raw fd, no buffering)."""
    keys = []
    while True:
        r, _, _ = select.select([fd], [], [], 0)
        if not r:
            break
        ch = os.read(fd, 1).decode("latin-1")
        if not ch:
            break
        keys.append(ch)
    return keys

def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    # Non-blocking writes so a full PTY buffer doesn't freeze the loop.
    fl = fcntl.fcntl(master_fd, fcntl.F_GETFL)
    fcntl.fcntl(master_fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

    print(f"Serial Device Fake: {slave_name}")
    print(f"Controls (hold, up/down): 1/q  2/w  3/e  4/r  5/t  6/y  7/u  |  a/z  s/x  d/c  f/v  g/b  h/n  j/m  |  Ctrl+C to quit")
    print()
    print(fmt_header(range(7)))
    print(fmt_values([2048.0] * 7))
    print(fmt_header(range(7, 14)))
    # Cursor rests at end of this line; refreshes use ANSI codes to rewrite
    # the two value rows (rows 2 and 4 relative to the header pair) in place.
    print(fmt_values([2048.0] * 7), end="", flush=True)

    # Use floats internally so fractional STEP accumulates correctly.
    vals = [2048.0] * N

    # last_seen[key] = perf_counter timestamp of most recent key event.
    last_seen: dict[str, float] = {}

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        t0 = time.perf_counter()
        k = 0
        next_t = t0

        while True:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
            now = time.perf_counter()

            # Refresh timestamps for every key seen this tick.
            for ch in read_keys_nonblocking(fd):
                if ch in KEYMAP:
                    last_seen[ch] = now

            # Apply step for every key still considered held.
            for ch, ts in list(last_seen.items()):
                if now - ts > HOLD_TIMEOUT:
                    del last_seen[ch]
                else:
                    idx, direction = KEYMAP[ch]
                    vals[idx] = wrap(vals[idx] + direction * STEP)

            t = time.perf_counter()
            t_us = INITIAL_TIME_US + int((t - t0) * 1_000_000)

            data = pack_frame(t_us, vals)
            try:
                n = os.write(master_fd, data)
                if n < len(data):
                    # Partial write: finish the remainder in blocking mode so the
                    # line stays intact and the reader never sees a split line.
                    fcntl.fcntl(master_fd, fcntl.F_SETFL, fl)
                    try:
                        os.write(master_fd, data[n:])
                    finally:
                        fcntl.fcntl(master_fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    raise

            # Overwrite both value rows in place (~20 Hz).
            # Cursor is at end of row 4 (bottom value row, no trailing newline).
            # \r\033[2A  -> start of row 2 (first value row)
            # \033[2B\r  -> start of row 4 (second value row)
            if k % DISPLAY_EVERY == 0:
                sys.stdout.write(
                    "\r\033[2A" + fmt_values(vals[:7]) +
                    "\033[2B\r" + fmt_values(vals[7:])
                )
                sys.stdout.flush()

            k += 1
            next_t = t0 + k * DT

    except KeyboardInterrupt:
        print()  # leave cursor on a fresh line after the values row
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == "__main__":
    main()