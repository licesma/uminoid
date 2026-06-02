"""Latest-wins frame hub shared between camera producers and the preview UI.

This replaces the role of the C++ PreviewServer, but without any network code:
camera recorders push the most recent RGB frame and IMU tilt here, and the
Gradio consumer reads them back. Gradio owns the transport, so there is no HTTP
server, no JPEG encoding, and no MJPEG plumbing on this side.

The push API intentionally mirrors the C++ PreviewServer (`push_rgb`,
`push_imu`) so the ported camera recorders can call it unchanged.
"""
from __future__ import annotations

import threading

import numpy as np


class Preview:
    """Thread-safe, single-slot frame buffer. Older frames are dropped."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._pitch_deg = 0.0
        self._roll_deg = 0.0

    def push_rgb(self, frame: np.ndarray) -> None:
        """Publish the latest frame (HxWx3 uint8; BGR in this pipeline).

        The caller must not mutate `frame` after handing it over; the hub keeps
        the reference rather than copying so the producer hot loop stays cheap.
        """
        with self._lock:
            self._frame = frame

    def push_imu(self, pitch_deg: float, roll_deg: float) -> None:
        with self._lock:
            self._pitch_deg = float(pitch_deg)
            self._roll_deg = float(roll_deg)

    def latest(self) -> tuple[np.ndarray | None, tuple[float, float]]:
        with self._lock:
            return self._frame, (self._pitch_deg, self._roll_deg)
