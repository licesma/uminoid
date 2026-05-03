import math

from joint_reader_constants import ENCODER_RESOLUTION, ENCODER_PRECISION_RAD


def convert_to_radian(encoder_value: float) -> float:
    """Map an encoder value to radians.

    0                   -> -π
    ENCODER_RESOLUTION  ->  π
    """
    return (encoder_value / ENCODER_RESOLUTION) * 2 * math.pi - math.pi


def circular_distance(high: float, low: float) -> float:
    """Angular distance from low to high on a 2π circle. Returns a value in [0, 2π)."""
    return high - low if low < high else 2 * math.pi + high - low


def to_robot_angle(
    bit_value: int,
    exo_lower: float,
    exo_upper: float,
    skeleton_ref: str,
    direction_aligned: bool,
    g1_lower: float,
    g1_upper: float,
) -> float | None:
    """Convert a raw encoder bit value to a robot joint angle.

    Mirrors the C++ UpperBodyReader::Eval + G1Controller::toG1Angle pipeline:
      1. bit_value  -> radians (centered at ENCODER_RESOLUTION/2)
      2. radians    -> net_angle (circular distance from the exo reference bound)
      3. net_angle  -> robot_angle (applied to the G1 reference bound)

    Returns None for invalid readings (sentinel value 5000 or out-of-range).

    Args:
        bit_value:        Raw encoder integer from the exoskeleton.
        exo_lower:        Lower bound of the exo joint (radians).
        exo_upper:        Upper bound of the exo joint (radians).
        skeleton_ref:     Which exo bound is the reference: "lower" or "upper".
        direction_aligned: If True, the G1 reference side matches skeleton_ref;
                           if False, it is the opposite side.
        g1_lower:         Lower bound of the G1 robot joint (radians).
        g1_upper:         Upper bound of the G1 robot joint (radians).
    """
    if bit_value == 5000:
        return None

    value = (bit_value - ENCODER_RESOLUTION / 2.0) * ENCODER_PRECISION_RAD

    if skeleton_ref == "lower":
        net_angle = circular_distance(value, exo_lower)
    else:
        net_angle = circular_distance(exo_upper, value)

    if direction_aligned:
        g1_ref = skeleton_ref
    else:
        g1_ref = "upper" if skeleton_ref == "lower" else "lower"

    if g1_ref == "lower":
        robot_angle = g1_lower + net_angle
    else:
        robot_angle = g1_upper - net_angle

    return max(g1_lower, min(g1_upper, robot_angle))
