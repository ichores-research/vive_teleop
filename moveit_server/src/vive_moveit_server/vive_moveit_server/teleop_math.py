"""Numerically stable math helpers shared by teleoperation controllers."""

import math

from geometry_msgs.msg import Quaternion


def normalize_quaternion(
    quaternion: Quaternion,
    minimum_norm: float = 1e-6,
) -> bool:
    """Normalize in place without overflowing on very large finite values."""

    components = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    if not all(math.isfinite(value) for value in components):
        return False

    scale = max(abs(value) for value in components)
    if scale == 0.0:
        return False

    scaled = tuple(value / scale for value in components)
    scaled_norm = math.sqrt(sum(value * value for value in scaled))
    if scale < minimum_norm and (scale * scaled_norm) < minimum_norm:
        return False

    quaternion.x = scaled[0] / scaled_norm
    quaternion.y = scaled[1] / scaled_norm
    quaternion.z = scaled[2] / scaled_norm
    quaternion.w = scaled[3] / scaled_norm
    return True
