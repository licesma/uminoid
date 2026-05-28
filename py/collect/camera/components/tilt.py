"""Camera tilt (pitch/roll) from a D435i accelerometer.

Extracted from the USB recorder so the math is reusable and unit-testable.
Mirrors the C++ derivation: D435i camera frame is +X right, +Y down, +Z forward,
and the camera is mounted rotated 180 deg about its forward axis. Values are
EMA-smoothed; at rest pitch and roll read ~0.
"""
from __future__ import annotations

import math


class TiltEstimator:
    def __init__(self, alpha: float = 0.1) -> None:
        self._alpha = alpha
        self.pitch_deg = 0.0
        self.roll_deg = 0.0

    def update(self, ax: float, ay: float, az: float) -> tuple[float, float]:
        ip = math.degrees(math.atan2(az, math.hypot(ax, ay)))
        ir = math.degrees(math.atan2(-ax, -ay))
        a = self._alpha
        self.pitch_deg = (1.0 - a) * self.pitch_deg + a * ip
        self.roll_deg = (1.0 - a) * self.roll_deg + a * ir
        return self.pitch_deg, self.roll_deg
