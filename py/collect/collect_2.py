#!/usr/bin/env python3
"""Temporary standalone camera runner for the Python camera migration.

Records ONE camera source to a session dir AND streams a live preview to the
browser over WebRTC (FastRTC). The source ("usb" or "zmq") + endpoint + server
port + WebRTC bitrate caps are read from config/collect2.yaml (no CLI flags).

Pipeline (both sources): capture/receive 1080p BGR -> full-res frame to the
preview hub -> WebRTC; the recorder downscales to 640x480 + JPEG-encodes for
storage. Standalone for now (collection_id fixed at 1, always recording); the
single Python orchestrator will drive collection_id/pause/stop later.

Run:
    python3 py/collect/collect_2.py
Then open http://localhost:7860 (or http://<tailscale-ip>:7860) and click
"Start Stream".

For source = "zmq", the robot must be running g1_files/realsense_jpeg_1080.py.

Networking note: WebRTC media negotiates its own path (ICE), separate from the
HTTP signaling. On localhost / same tailnet this works directly; for a relayed
viewer you may need a TURN server via RTC_CONFIGURATION below.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml
from fastrtc import Stream

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # put py/ on sys.path
from collect.camera.components.preview import Preview
from collect.camera.usb_camera_recorder import (
    CAPTURE_HEIGHT,
    CAPTURE_WIDTH,
    UsbCameraRecorder,
)
from collect.camera.zmq_camera_recorder import ZmqCameraRecorder
from paths import DATA_DIR, ROOT_DIR
from utils.recording_label import generate_recording_label

FPS = 30
CONFIG_PATH = ROOT_DIR / "config" / "collect2.yaml"

# Leave as None for localhost / same tailnet. For a relayed viewer, supply
# STUN/TURN here, e.g. {"iceServers": [{"urls": "stun:stun.l.google.com:19302"}]}
# or FastRTC's get_cloudflare_turn_credentials()/get_hf_turn_credentials().
RTC_CONFIGURATION = None


def _load_config() -> dict:
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f) or {}


def _raise_webrtc_bitrate(start_bps: int, target_bps: int) -> None:
    try:
        from aiortc.codecs import h264, vpx

        for mod in (h264, vpx):
            mod.DEFAULT_BITRATE = start_bps
            mod.MAX_BITRATE = target_bps
    except Exception as e:  # noqa: BLE001
        print(f"[collect_2] could not raise webrtc bitrate: {e}", flush=True)


def make_handler(preview: Preview):
    """Generator handed to FastRTC: yields the latest frame at the target rate.

    The preview hub holds BGR (the JPEG-everywhere convention) and FastRTC encodes
    its outbound track from bgr24, so yield frames directly — no conversion.
    """
    period = 1.0 / FPS
    black = np.zeros((CAPTURE_HEIGHT, CAPTURE_WIDTH, 3), dtype=np.uint8)

    def handler():
        while True:
            frame, _ = preview.latest()
            yield frame if frame is not None else black
            time.sleep(period)

    return handler


def _print_access_urls(port: int) -> None:
    # Gradio prints "Running on http://0.0.0.0:7860" — but 0.0.0.0 is only a
    # bind address; a remote browser cannot connect to it. Print the real URLs.
    import subprocess

    print(f"[collect_2] open locally:  http://127.0.0.1:{port}", flush=True)
    try:
        out = subprocess.run(
            ["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=3
        ).stdout.strip().splitlines()
        if out:
            print(
                f"[collect_2] open on laptop: http://{out[0]}:{port}  (NOT 0.0.0.0)",
                flush=True,
            )
    except Exception:
        pass


def _build_recorder(cfg: dict, label: str, preview: Preview):
    cam = cfg.get("camera", {})
    src = cam.get("source", "usb")
    if src == "usb":
        return UsbCameraRecorder(label, preview=preview)
    if src == "zmq":
        return ZmqCameraRecorder(label, preview=preview, endpoint=cam["zmq_endpoint"])
    raise ValueError(f"unknown camera.source in {CONFIG_PATH}: {src!r}")


def main() -> None:
    cfg = _load_config()
    cam = cfg.get("camera", {})
    source = cam.get("source", "usb")
    port = (cfg.get("server") or {}).get("port", 7860)
    preview_cfg = cfg.get("preview") or {}

    label = generate_recording_label()
    print(f"[collect_2] config: {CONFIG_PATH}  | source: {source}", flush=True)
    if source == "zmq":
        print(f"[collect_2] zmq endpoint: {cam['zmq_endpoint']}", flush=True)
    print(f"[collect_2] recording session: {DATA_DIR}/{label}", flush=True)
    _print_access_urls(port)
    _raise_webrtc_bitrate(
        preview_cfg.get("start_bitrate_bps", 6_000_000),
        preview_cfg.get("target_bitrate_bps", 10_000_000),
    )

    preview = Preview()
    recorder = _build_recorder(cfg, label, preview)

    stop = threading.Event()
    rec_thread = threading.Thread(
        target=recorder.collect_loop,
        args=(lambda: 1, stop.is_set, lambda: False),  # collection_id=1, always recording
        daemon=True,
    )
    rec_thread.start()

    stream = Stream(
        handler=make_handler(preview),
        modality="video",
        mode="receive",
        rtc_configuration=RTC_CONFIGURATION,
    )
    try:
        stream.ui.launch(server_name="0.0.0.0", server_port=port)
    finally:
        stop.set()
        rec_thread.join(timeout=10)


if __name__ == "__main__":
    main()
