#!/usr/bin/env python3
"""Temporary FastRTC preview consumer for the Python camera migration.

Replaces collect_with_preview.py's terminal/sixel preview with a browser-based
WebRTC view (FastRTC). WebRTC gives inter-frame compression + adaptive bitrate,
which is what a constrained/relayed (DERP) link needs for 1080p30.

For now it streams a synthetic 1080p test pattern so the streaming path can be
validated *before* the real cameras are ported. Once
py/camera/usb_camera_recorder.py exists, swap `synthetic_producer` for the real
recorder feeding the same Preview hub; the streaming layer below stays identical.

Run:
    python3 py/collect_2.py
Then open http://localhost:7860 and click "Start Stream".

Networking note: WebRTC media negotiates its own path (ICE), separate from the
HTTP signaling. On localhost this just works. For a remote viewer over Tailscale
or an SSH port-forward you will likely need a TURN relay or an explicit
rtc_configuration — see RTC_CONFIGURATION below.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from fastrtc import Stream

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # put py/ on sys.path
from collect.camera.preview import Preview

# Stream geometry. Drop to 1280x720 @ 15 to see how a constrained link copes.
WIDTH = 1920
HEIGHT = 1080
FPS = 30
SERVER_PORT = 7860

# Leave as None for localhost. For a remote viewer, supply STUN/TURN here, e.g.
# {"iceServers": [{"urls": "stun:stun.l.google.com:19302"}]} — or use FastRTC's
# get_cloudflare_turn_credentials()/get_hf_turn_credentials() helpers.
RTC_CONFIGURATION = None


def synthetic_producer(preview: Preview, stop: threading.Event) -> None:
    """Generate a moving test pattern with an on-frame clock and frame counter.

    The wall-clock readout and counter let you eyeball end-to-end latency and
    dropped frames directly in the browser while probing the link.
    """
    period = 1.0 / FPS
    n = 0
    while not stop.is_set():
        t0 = time.monotonic()

        frame = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
        # Horizontal gradient background (gives the codec real content to chew).
        frame[:, :, 0] = np.linspace(0, 255, WIDTH, dtype=np.uint8)[None, :]
        frame[:, :, 1] = 40
        frame[:, :, 2] = np.linspace(255, 0, WIDTH, dtype=np.uint8)[None, :]

        # Moving vertical bar.
        x = int((n * 12) % WIDTH)
        cv2.rectangle(frame, (x, 0), (min(x + 80, WIDTH), HEIGHT), (255, 255, 255), -1)

        pitch = 15.0 * np.sin(n / 30.0)
        roll = 10.0 * np.cos(n / 40.0)

        # Overlays: wall clock (ms), frame index, geometry, fake IMU.
        ms = int(time.time() * 1000) % 100000
        cv2.putText(frame, f"{ms:05d} ms", (40, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 3.5, (0, 0, 0), 8)
        cv2.putText(frame, f"frame {n}", (40, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.2, (0, 0, 0), 5)
        cv2.putText(frame, f"{WIDTH}x{HEIGHT} @ {FPS}fps", (40, 360),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.2, (0, 0, 0), 5)
        cv2.putText(frame, f"pitch {pitch:+6.1f}  roll {roll:+6.1f}", (40, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.2, (0, 0, 0), 5)

        preview.push_rgb(frame)
        preview.push_imu(pitch, roll)

        n += 1
        dt = time.monotonic() - t0
        if dt < period:
            time.sleep(period - dt)


def make_handler(preview: Preview):
    """Generator handed to FastRTC: yields the latest frame at the target rate.

    FastRTC encodes each yielded RGB ndarray into the outbound WebRTC video track.
    """
    period = 1.0 / FPS
    black = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    def handler():
        while True:
            frame, _ = preview.latest()
            yield frame if frame is not None else black
            time.sleep(period)

    return handler


def main() -> None:
    preview = Preview()
    stop = threading.Event()
    producer = threading.Thread(
        target=synthetic_producer, args=(preview, stop), daemon=True
    )
    producer.start()

    stream = Stream(
        handler=make_handler(preview),
        modality="video",
        mode="receive",
        rtc_configuration=RTC_CONFIGURATION,
    )
    try:
        stream.ui.launch(server_name="0.0.0.0", server_port=SERVER_PORT)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
