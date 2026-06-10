import cv2
import numpy as np

CHANNELS = 3  # RGB8
# Lossless PNG buys nothing here: frames are re-encoded into a lossy H.264 mp4
# downstream, so JPEG q95 is visually indistinguishable post-encode and far faster.
JPEG_QUALITY = 95


# Transform raw RGB8 bytes into JPEG-encoded bytes.
def rgb_to_jpeg(raw: bytes, width: int, height: int) -> bytes:
    expected_size = width * height * CHANNELS
    if len(raw) != expected_size:
        raise ValueError(
            f"Expected {expected_size} bytes for a {height}x{width} RGB frame, got {len(raw)}"
        )

    rgb = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, CHANNELS))
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise RuntimeError("cv2.imencode failed for JPEG")
    return encoded.tobytes()
