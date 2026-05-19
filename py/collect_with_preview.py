#!/usr/bin/env python3
"""
Run ./collect with an MJPEG sixel preview painted above it in the same
terminal. Top region: video. Bottom region: ./collect TUI (owns the keyboard).

Requires: img2sixel (apt install libsixel-bin) and an iTerm2/kitty/WezTerm-class
terminal that renders sixel.
"""
import fcntl
import json
import os
import pty
import select
import shutil
import signal
import struct
import subprocess
import sys
import termios
import threading
import tty
import urllib.request

REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COLLECT_BIN = os.path.join(REPO_ROOT, "cpp", "build", "collect")
STREAM_URL  = "http://127.0.0.1:8080/stream.mjpg"
IMU_URL     = "http://127.0.0.1:8080/imu"

# Approximate terminal cell size in pixels (only used to size the sixel image).
# iTerm2 on Retina displays uses much larger cells than a plain xterm; tune
# these if the image overflows or undershoots the dedicated pane.
CELL_W_PX = 20
CELL_H_PX = 36


def write_stdout(data: bytes) -> None:
    os.write(sys.stdout.fileno(), data)


class LatestFrame:
    """Single-slot, latest-wins frame buffer. Older frames are dropped."""
    def __init__(self):
        self._lock  = threading.Lock()
        self._cond  = threading.Condition(self._lock)
        self._jpeg  = None
        self._seq   = 0

    def put(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify()

    def get(self, last_seen: int, stop: threading.Event):
        with self._cond:
            while self._seq == last_seen and not stop.is_set():
                self._cond.wait(timeout=0.2)
            return self._jpeg, self._seq


def producer_loop(url: str, latest: LatestFrame, stop: threading.Event) -> None:
    """Read MJPEG as fast as the network delivers; always drop into `latest`."""
    while not stop.is_set():
        try:
            with urllib.request.urlopen(url, timeout=2) as resp:
                buf = b""
                while not stop.is_set():
                    chunk = resp.read(16384)
                    if not chunk:
                        break
                    buf += chunk
                    # Drain all complete JPEGs in buffer, keep only the last.
                    last_jpeg = None
                    while True:
                        s = buf.find(b"\xff\xd8")
                        if s < 0:
                            break
                        e = buf.find(b"\xff\xd9", s)
                        if e < 0:
                            break
                        last_jpeg = buf[s:e + 2]
                        buf = buf[e + 2:]
                    if last_jpeg is not None:
                        latest.put(last_jpeg)
        except Exception:
            if stop.wait(0.5):
                return


def video_loop(top_row: int, video_rows: int, cols: int, stop: threading.Event) -> None:
    # Fit a 4:3 frame (source is 320x240) inside the pane without stretching.
    # img2sixel stretches when both -w and -h are given, so we compute an
    # aspect-preserving target ourselves.
    pane_w = cols * CELL_W_PX
    pane_h = video_rows * CELL_H_PX
    src_aspect = 320 / 240
    if pane_w / max(1, pane_h) > src_aspect:
        h_px = pane_h
        w_px = int(h_px * src_aspect)
    else:
        w_px = pane_w
        h_px = int(w_px / src_aspect)

    latest = LatestFrame()
    producer = threading.Thread(
        target=producer_loop, args=(STREAM_URL, latest, stop), daemon=True,
    )
    producer.start()

    seen = 0
    while not stop.is_set():
        jpeg, seq = latest.get(seen, stop)
        if jpeg is None:
            continue
        seen = seq
        try:
            r = subprocess.run(
                ["img2sixel", "-w", str(w_px), "-h", str(h_px)],
                input=jpeg, capture_output=True, timeout=2,
            )
            if r.returncode != 0 or not r.stdout:
                continue
            sixel = r.stdout
        except Exception:
            continue
        out  = b"\033[3J"                              # clear scrollback
        out += b"\0337"                               # save cursor
        out += f"\033[{top_row};1H".encode()          # move to top of video area
        out += sixel                                   # paint
        out += b"\0338"                               # restore cursor
        try:
            write_stdout(out)
        except OSError:
            break


def imu_loop(row: int, cols: int, stop: threading.Event) -> None:
    """Poll /imu and paint a single status line at `row` (1-based). The line
    sits between the video pane and the collect TUI, outside the TUI's scroll
    region, so repaints don't fight the child process."""
    while not stop.is_set():
        try:
            with urllib.request.urlopen(IMU_URL, timeout=1) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            pitch = float(data.get("pitch_deg", 0.0))
            roll  = float(data.get("roll_deg", 0.0))
            text  = f"  Roll: {roll:+6.1f}°    Pitch: {pitch:+6.1f}°"
        except Exception:
            text = "  Roll:   ---       Pitch:   ---"
        text = text[:cols].ljust(cols)
        out  = b"\0337"                           # save cursor
        out += f"\033[{row};1H".encode()         # move to status row
        out += b"\033[2K"                         # clear line
        out += text.encode("utf-8")
        out += b"\0338"                           # restore cursor
        try:
            write_stdout(out)
        except OSError:
            break
        if stop.wait(0.2):
            return


def set_pty_size(fd: int, rows: int, cols: int) -> None:
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))


def main() -> int:
    if not os.path.isfile(COLLECT_BIN) or not os.access(COLLECT_BIN, os.X_OK):
        sys.stderr.write(f"collect binary not found at {COLLECT_BIN}\n")
        return 1

    term_cols, term_rows = shutil.get_terminal_size()
    video_rows = max(8, term_rows // 2)
    ui_top     = video_rows + 2   # +1 would sit flush; extra row avoids overflow
    ui_rows    = term_rows - video_rows - 1

    # Fork ./collect under a pty so its RawMode and in-place redraws keep working.
    pid, master_fd = pty.fork()
    if pid == 0:
        os.chdir(os.path.dirname(COLLECT_BIN))
        os.execvp(COLLECT_BIN, [COLLECT_BIN])
    set_pty_size(master_fd, ui_rows, term_cols)

    # Save our terminal state and drop into raw mode so keys pass through untouched.
    stdin_fd = sys.stdin.fileno()
    orig_attr = termios.tcgetattr(stdin_fd)
    tty.setraw(stdin_fd)

    # Clear screen, carve scroll region for the UI, park cursor inside it,
    # and hide the cursor for the whole session (video repainting at 10fps
    # otherwise makes it blink frantically).
    write_stdout(b"\033[2J")                              # clear
    write_stdout(f"\033[{ui_top};{term_rows}r".encode()) # DECSTBM
    write_stdout(f"\033[{ui_top};1H".encode())           # cursor into UI area
    write_stdout(b"\033[?25l")                            # hide cursor (DECTCEM)

    stop = threading.Event()
    tail = bytearray()
    TAIL_MAX = 16384
    exit_status = 0

    def on_winch(_signum, _frame):
        # Keep the child pty in sync with terminal size changes.
        new_cols, new_rows = shutil.get_terminal_size()
        nonlocal video_rows
        video_rows = max(8, new_rows // 2)
        set_pty_size(master_fd, new_rows - video_rows, new_cols)
    signal.signal(signal.SIGWINCH, on_winch)

    vt = threading.Thread(
        target=video_loop,
        args=(1, video_rows, term_cols, stop),
        daemon=True,
    )
    vt.start()

    imu_t = threading.Thread(
        target=imu_loop,
        args=(video_rows + 1, term_cols, stop),
        daemon=True,
    )
    imu_t.start()

    try:
        while True:
            try:
                r, _, _ = select.select([stdin_fd, master_fd], [], [], 0.1)
            except InterruptedError:
                continue
            if stdin_fd in r:
                data = os.read(stdin_fd, 1024)
                if data:
                    os.write(master_fd, data)
            if master_fd in r:
                try:
                    data = os.read(master_fd, 4096)
                except OSError:
                    break
                if not data:
                    break
                write_stdout(data)
                tail.extend(data)
                if len(tail) > TAIL_MAX:
                    del tail[:len(tail) - TAIL_MAX]
            # Reap collect if it exited.
            try:
                wpid, status = os.waitpid(pid, os.WNOHANG)
                if wpid == pid:
                    exit_status = status
                    break
            except ChildProcessError:
                break
    finally:
        stop.set()
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        write_stdout(b"\033[?25h")              # show cursor
        write_stdout(b"\033[r")                 # reset scroll region
        write_stdout(b"\033[2J\033[H")          # clear + home
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, orig_attr)

    rc = os.waitstatus_to_exitcode(exit_status) if exit_status else 0
    if rc != 0 and tail:
        sys.stderr.write(f"\ncollect exited with status {rc}; last output:\n")
        sys.stderr.buffer.write(bytes(tail))
        sys.stderr.write("\n")
    return rc


if __name__ == "__main__":
    sys.exit(main())
