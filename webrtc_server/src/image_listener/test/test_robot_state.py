import math
from types import SimpleNamespace

from image_listener.robot_state import _transform_is_valid


def _transform(
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z),
            rotation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
        )
    )


def test_finite_transform_with_nonzero_quaternion_is_valid() -> None:
    assert _transform_is_valid(_transform()) is True


def test_non_finite_transform_is_invalid() -> None:
    assert _transform_is_valid(_transform(x=math.nan)) is False


def test_zero_length_transform_quaternion_is_invalid() -> None:
    assert _transform_is_valid(_transform(qw=0.0)) is False


def test_non_unit_transform_quaternion_is_invalid() -> None:
    assert _transform_is_valid(_transform(qw=10.0)) is False
