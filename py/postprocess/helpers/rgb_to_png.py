import cv2
import numpy as np

CHANNELS = 3  # RGB8


# Transform raw RGB8 bytes into PNG-encoded bytes.
def rgb_to_png(raw: bytes, width: int, height: int) -> bytes:
    expected_size = width * height * CHANNELS
    if len(raw) != expected_size:
        raise ValueError(
            f"Expected {expected_size} bytes for a {height}x{width} RGB frame, got {len(raw)}"
        )

    rgb = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, CHANNELS))
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("cv2.imencode failed for PNG")
    return encoded.tobytes()
