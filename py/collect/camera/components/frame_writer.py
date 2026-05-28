"""Off-thread frame + CSV writer for camera recorders.

A background thread drains a queue and does all the disk I/O (one raw frame file
plus one CSV row per frame), so the capture/receive loop never blocks on disk —
the key to holding 30 fps. The capture thread only calls submit().

Works in CPython despite the GIL because file write/flush release the GIL while
blocked in the syscall.

`to_bytes` converts a queued payload into the raw bytes to write — it runs on the
writer thread, so per-source work like downscaling stays off the capture loop.
"""
from __future__ import annotations

import os
import queue
import threading
from typing import Any, Callable, Optional

from utils.csv_saver import CsvSaver


class FrameWriter:
    def __init__(
        self,
        output_dir: str,
        csv_header: str,
        to_bytes: Callable[[Any], bytes],
        frame_ext: str = ".jpg",
        flush_every: int = 1000,
    ) -> None:
        self._frames_dir = os.path.join(output_dir, "frames")
        os.makedirs(self._frames_dir, exist_ok=True)
        self._csv = CsvSaver(
            os.path.join(output_dir, "camera.csv"), csv_header, flush_every
        )
        self._to_bytes = to_bytes
        self._ext = frame_ext
        self._q: "queue.Queue[Optional[tuple]]" = queue.Queue()
        self._thread = threading.Thread(
            target=self._loop, name="frame-writer", daemon=True
        )
        self._thread.start()

    def submit(self, frame_number: int, csv_line: str, payload: Any) -> None:
        """Non-blocking hand-off from the capture loop to the writer thread."""
        self._q.put((frame_number, csv_line, payload))

    def close(self) -> None:
        """Flush everything already queued, then stop the writer and CSV."""
        self._q.put(None)
        self._thread.join()
        self._csv.close()

    def _loop(self) -> None:
        while True:
            item = self._q.get()
            if item is None:  # shutdown sentinel
                break
            frame_number, csv_line, payload = item
            self._csv.write_line(csv_line)
            fn = os.path.join(self._frames_dir, f"frame_{frame_number:06d}{self._ext}")
            with open(fn, "wb") as f:
                f.write(self._to_bytes(payload))
