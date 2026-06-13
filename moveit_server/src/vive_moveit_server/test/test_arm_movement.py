import math

import pytest
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool

from vive_moveit_server.arm_movement import ArmMovementMixin


def _pose(
    x: float,
    y: float,
    z: float,
    orientation: Quaternion | None = None,
) -> PoseStamped:
    message = PoseStamped()
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z
    message.pose.orientation = orientation or Quaternion(w=1.0)
    return message


def test_controller_delta_is_applied_to_live_robot_anchor() -> None:
    movement = object.__new__(ArmMovementMixin)
    movement.deadman_controller_anchor = _pose(10.0, 20.0, 30.0)
    movement.deadman_robot_anchor = _pose(0.5, -0.2, 0.8)

    target = movement._map_controller_delta_to_robot(
        _pose(10.1, 19.7, 30.2)
    )

    assert (
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
    ) == pytest.approx((0.6, -0.5, 1.0))


def test_hand_position_scale_changes_displacement_per_axis() -> None:
    movement = object.__new__(ArmMovementMixin)
    movement.hand_position_scale = [2.0, 0.5, -1.0]

    target = movement._scale_hand_target(_pose(0.1, -0.2, 0.3))

    assert (
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
    ) == pytest.approx((0.2, -0.1, -0.3))


def test_controller_rotation_delta_is_applied_to_robot_anchor() -> None:
    movement = object.__new__(ArmMovementMixin)
    movement.deadman_controller_anchor = _pose(0.0, 0.0, 0.0)
    movement.deadman_robot_anchor = _pose(0.0, 0.0, 0.0)
    half_angle = math.pi / 4.0

    target = movement._map_controller_delta_to_robot(
        _pose(
            0.0,
            0.0,
            0.0,
            Quaternion(z=math.sin(half_angle), w=math.cos(half_angle)),
        )
    )

    assert target.pose.orientation.z == pytest.approx(math.sin(half_angle))
    assert target.pose.orientation.w == pytest.approx(math.cos(half_angle))


def test_deadman_release_clears_clutch_immediately() -> None:
    movement = object.__new__(ArmMovementMixin)
    movement.hand_target_active = True
    movement.pending_hand_target = _pose(1.0, 2.0, 3.0)
    movement.last_commanded_target = _pose(1.0, 2.0, 3.0)
    movement.deadman_controller_anchor = _pose(1.0, 2.0, 3.0)
    movement.deadman_robot_anchor = _pose(0.5, 0.0, 0.8)
    movement.last_hand_target_received_sec = 1.0
    movement.get_logger = lambda: type(
        "Logger",
        (),
        {"info": lambda self, message: None},
    )()

    movement._on_hand_target_active(Bool(data=False))

    assert movement.hand_target_active is False
    assert movement.pending_hand_target is None
    assert movement.deadman_controller_anchor is None
    assert movement.deadman_robot_anchor is None
