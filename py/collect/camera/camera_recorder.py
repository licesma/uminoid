"""Camera recorder base: owns the whole pipeline; sources only acquire frames.

Port of cpp/camera/camera_recorder.hpp plus all the shared logic. The base runs
the entire collect_loop — open writer, pull frames, push preview, honor pause,
record, teardown — and the storage path (downscale capture-res -> 640x480 +
JPEG). A source subclass implements only:

  _get_frame() -> Frame | None   (required): acquire one capture-res BGR frame;
        return None to skip this tick (transient timeout / bad frame); raise
        RecorderStop to end the loop (after reporting a fatal source error).
  _setup()    (optional): per-run init (e.g. USB warmup, ZMQ poller).
  _teardown() (optional): per-run cleanup (e.g. pipe.stop, socket close).

Frames are BGR throughout (cv2 / robot JPEG / deploy all use BGR), so there are
no RGB<->BGR conversions. On disk: frame_XXXXXX.jpg (480p) + camera.csv, with
host_timestamp stamped at capture time.
"""
from __future__ import annotations

import abc
import os
import sys
from dataclasses import dataclass
from typing import Any, Callable, Optional

import cv2

from collect.camera.components.frame_writer import FrameWriter
from paths import DATA_DIR
from utils.host_time import ts

# Storage geometry + quality.
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
STORAGE_JPEG_QUALITY = 90

CSV_HEADER = (
    "collection_id,frame_number,camera_timestamp_ms,host_timestamp,pitch_deg,roll_deg"
)


@dataclass
class Frame:
    """One acquired frame handed from a source to the base loop."""

    img: Any          # capture-res BGR ndarray
    camera_ts: str    # camera_timestamp_ms CSV field (USB: device ts; ZMQ: "-1")
    pitch_deg: float
    roll_deg: float


class RecorderStop(Exception):
    """Raised by _get_frame to end collect_loop (fatal source error reported)."""


class CameraRecorder(abc.ABC):
    def __init__(
        self,
        recording_label: str,
        on_error: Optional[Callable[[str], None]] = None,
        preview: Optional[Any] = None,
    ) -> None:
        self._on_error = on_error or (lambda m: print(m, file=sys.stderr))
        self._preview = preview
        self._output_dir = os.path.join(str(DATA_DIR), recording_label)
        self._writer: Optional[FrameWriter] = None
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), STORAGE_JPEG_QUALITY]

    # ------------------------------------------------------------------ loop
    def collect_loop(
        self,
        collection_id: Callable[[], int],
        stop: Callable[[], bool],
        pause: Callable[[], bool],
    ) -> None:
        self._writer = FrameWriter(
            self._output_dir, CSV_HEADER, self._to_storage_bytes, frame_ext=".jpg"
        )
        try:
            self._setup()
            frame_count = 0
            while not stop():
                try:
                    frame = self._get_frame()
                except RecorderStop:
                    break
                if frame is None:
                    continue
                if self._preview:
                    self._preview.push_rgb(frame.img)
                    self._preview.push_imu(frame.pitch_deg, frame.roll_deg)
                if pause():
                    continue
                self._record(
                    frame_count,
                    collection_id(),
                    frame.camera_ts,
                    frame.pitch_deg,
                    frame.roll_deg,
                    frame.img,
                )
                frame_count += 1
        except RecorderStop:
            pass  # source signaled a clean stop during _setup or elsewhere
        except Exception as e:  # noqa: BLE001 - report and exit
            self._on_error(f"[{type(self).__name__}] {e}")
        finally:
            if self._writer is not None:
                self._writer.close()
            self._teardown()

    # ------------------------------------------------------------ source hooks
    @abc.abstractmethod
    def _get_frame(self) -> Optional[Frame]:
        """Acquire one capture-res BGR frame, None to skip, or raise RecorderStop."""
        raise NotImplementedError

    def _setup(self) -> None:
        """Per-run init before the loop (optional)."""

    def _teardown(self) -> None:
        """Per-run cleanup after the loop (optional)."""

    # --------------------------------------------------------------- internals
    def _to_storage_bytes(self, frame: Any) -> bytes:
        # Capture-res BGR -> downscaled 640x480 JPEG. Runs on the writer thread.
        small = cv2.resize(
            frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_AREA
        )
        ok, jpeg = cv2.imencode(".jpg", small, self._jpeg_params)
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return jpeg.tobytes()

    def _record(
        self,
        frame_number: int,
        collection_id: int,
        camera_ts: str,
        pitch_deg: float,
        roll_deg: float,
        frame: Any,
    ) -> None:
        # host_timestamp stamped here (capture thread), not on the writer thread.
        csv_line = (
            f"{collection_id},{frame_number},{camera_ts},{ts()},"
            f"{pitch_deg:.2f},{roll_deg:.2f}"
        )
        assert self._writer is not None
        self._writer.submit(frame_number, csv_line, frame)
