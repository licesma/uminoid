"""USB (Intel RealSense) camera source.

Captures 1080p BGR8 @30fps + the D435i accelerometer; derives EMA pitch/roll via
tilt.TiltEstimator. Only the source-specific hooks live here — the base
(CameraRecorder) runs the loop, preview, storage (downscale 480 + JPEG), and CSV.
"""
from __future__ import annotations

import sys
from typing import Callable, Optional

import numpy as np
import pyrealsense2 as rs

from collect.camera.camera_recorder import CameraRecorder, Frame, RecorderStop
from collect.camera.components.preview import Preview
from collect.camera.components.tilt import TiltEstimator

FRAMERATE = 30
ACCEL_FRAMERATE = 100  # D435i: 100 Hz works across firmwares
CAPTURE_WIDTH = 1920   # capture 1080p for the preview; base downscales for storage
CAPTURE_HEIGHT = 1080
FRAME_TIMEOUT_MS = 5000
WARMUP_COUNT = 30


class UsbCameraRecorder(CameraRecorder):
    def __init__(
        self,
        recording_label: str,
        on_error: Optional[Callable[[str], None]] = None,
        preview: Optional[Preview] = None,
    ) -> None:
        super().__init__(recording_label, on_error, preview)
        self._ctx = rs.context()
        self._pipe = rs.pipeline(self._ctx)
        self._tilt = TiltEstimator()
        self._pipeline_started = False
        self._has_accel = self._start_pipeline()

    def _start_pipeline(self) -> bool:
        if len(self._ctx.query_devices()) == 0:
            self._on_error("[Camera] No camera connected")
            return False
        # bgr8 to match the JPEG-everywhere BGR convention. Prefer color + accel;
        # fall back to color only if the IMU isn't available.
        try:
            cfg = rs.config()
            cfg.enable_stream(
                rs.stream.color, CAPTURE_WIDTH, CAPTURE_HEIGHT, rs.format.bgr8, FRAMERATE
            )
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, ACCEL_FRAMERATE)
            self._pipe.start(cfg)
            self._pipeline_started = True
            return True
        except Exception as e:
            print(f"[Camera] accel start failed: {e}", file=sys.stderr)
        cfg = rs.config()
        cfg.enable_stream(
            rs.stream.color, CAPTURE_WIDTH, CAPTURE_HEIGHT, rs.format.bgr8, FRAMERATE
        )
        self._pipe.start(cfg)
        self._pipeline_started = True
        print("[Camera] IMU unavailable — pitch/roll will read 0", file=sys.stderr)
        return False

    def _setup(self) -> None:
        if not self._pipeline_started:
            # _start_pipeline already called on_error; exit the loop cleanly.
            raise RecorderStop
        for _ in range(WARMUP_COUNT):
            self._pipe.try_wait_for_frames(FRAME_TIMEOUT_MS)

    def _get_frame(self) -> Optional[Frame]:
        ok, frames = self._pipe.try_wait_for_frames(FRAME_TIMEOUT_MS)
        if not ok:
            if len(self._ctx.query_devices()) == 0:
                self._on_error("[Camera] Camera disconnected")
                raise RecorderStop
            return None
        color = frames.get_color_frame()
        if not color:
            return None

        accel = frames.first_or_default(rs.stream.accel)
        if accel:
            a = accel.as_motion_frame().get_motion_data()
            pitch, roll = self._tilt.update(a.x, a.y, a.z)
        else:
            pitch, roll = self._tilt.pitch_deg, self._tilt.roll_deg

        # Copy out of the RealSense buffer (recycled on the next wait).
        img = np.asanyarray(color.get_data()).copy()  # capture-res BGR
        return Frame(img, f"{color.get_timestamp():.3f}", pitch, roll)

    def _teardown(self) -> None:
        try:
            self._pipe.stop()
        except Exception:
            pass
