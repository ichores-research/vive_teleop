import math

import pytest
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import JointState

from vive_moveit_server.vive_moveit_server import ViveMoveItServer


def test_head_quaternion_converts_to_expected_pan_and_tilt() -> None:
    server = object.__new__(ViveMoveItServer)
    server.head_pan_sign = 1.0
    server.head_tilt_sign = 1.0
    half_angle = math.pi / 4.0
    message = PoseStamped()
    message.pose.orientation = Quaternion(
        y=math.sin(half_angle),
        w=math.cos(half_angle),
    )

    pan_tilt = server._head_pose_to_pan_tilt(message)

    assert pan_tilt == pytest.approx((math.pi / 2.0, 0.0))


def test_non_finite_head_quaternion_is_rejected() -> None:
    server = object.__new__(ViveMoveItServer)
    server._warn_throttled = lambda *args: None
    message = PoseStamped()
    message.pose.orientation = Quaternion(x=math.nan, w=1.0)

    assert server._head_pose_to_pan_tilt(message) is None


def test_non_finite_joint_positions_are_not_cached() -> None:
    server = object.__new__(ViveMoveItServer)
    server.current_joint_positions = {}
    server.received_joint_state = False
    server.get_logger = lambda: type(
        "Logger",
        (),
        {"info": lambda self, message: None},
    )()

    server._on_joint_state(
        JointState(
            name=["valid_joint", "invalid_joint"],
            position=[0.25, math.nan],
        )
    )

    assert server.current_joint_positions == {"valid_joint": 0.25}
    assert server.received_joint_state is True
