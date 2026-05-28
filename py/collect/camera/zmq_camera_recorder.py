"""ZMQ (G1 head camera) source.

Consumes the robot's MJPEG 1080p bridge (g1_files/realsense_jpeg_1080.py): each
ZMQ message is JPEG bytes + a fixed 8-byte trailer (little-endian float32 pitch,
float32 roll). The PULL socket uses CONFLATE (latest-wins). Only the
source-specific hooks live here — the base runs the loop, preview, storage
(downscale 480 + JPEG), and CSV. camera_timestamp_ms is -1 (no device ts).
"""
from __future__ import annotations

import struct
import time
from typing import Callable, Optional

import cv2
import numpy as np
import zmq

from collect.camera.camera_recorder import CameraRecorder, Frame, RecorderStop
from collect.camera.components.preview import Preview

ENDPOINT = "tcp://192.168.123.164:5556"
RECV_TIMEOUT_MS = 2000
IMU_BYTES = 8  # trailing float32 pitch + float32 roll


class ZmqCameraRecorder(CameraRecorder):
    def __init__(
        self,
        recording_label: str,
        on_error: Optional[Callable[[str], None]] = None,
        preview: Optional[Preview] = None,
        endpoint: str = ENDPOINT,
    ) -> None:
        super().__init__(recording_label, on_error, preview)
        self._endpoint = endpoint
        self._ctx = zmq.Context(1)
        self._sock = self._ctx.socket(zmq.PULL)
        self._sock.setsockopt(zmq.CONFLATE, 1)  # latest-wins; must precede connect
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(endpoint)

    def _setup(self) -> None:
        self._poller = zmq.Poller()
        self._poller.register(self._sock, zmq.POLLIN)
        self._last_packet = time.monotonic()

    def _get_frame(self) -> Optional[Frame]:
        socks = dict(self._poller.poll(RECV_TIMEOUT_MS))
        now = time.monotonic()
        if self._sock not in socks:
            if now - self._last_packet > RECV_TIMEOUT_MS / 1000.0:
                self._on_error(
                    f"[ZmqCamera] No data from {self._endpoint} for {RECV_TIMEOUT_MS}ms"
                )
                raise RecorderStop
            return None

        msg = self._sock.recv()
        self._last_packet = now
        if len(msg) <= IMU_BYTES:
            self._on_error(f"[ZmqCamera] Short message: {len(msg)} bytes")
            raise RecorderStop

        # Single payload: JPEG + 8-byte IMU trailer.
        pitch, roll = struct.unpack("<ff", msg[-IMU_BYTES:])
        img = cv2.imdecode(
            np.frombuffer(msg[:-IMU_BYTES], dtype=np.uint8), cv2.IMREAD_COLOR
        )  # BGR, capture resolution (1080)
        if img is None:
            self._on_error("[ZmqCamera] JPEG decode failed")
            return None
        return Frame(img, "-1", pitch, roll)

    def _teardown(self) -> None:
        try:
            self._sock.close()
        except Exception:
            pass
        try:
            self._ctx.term()
        except Exception:
            pass
