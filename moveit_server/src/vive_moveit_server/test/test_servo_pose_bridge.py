import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool

from vive_moveit_server.servo_pose_bridge import (
    ServoPoseBridge,
    _clamp_vector,
    _normalize_quaternion,
    _pose_feedback,
)


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _Clock:
    class _Now:
        @staticmethod
        def to_msg() -> Time:
            return Time(sec=12)

    @staticmethod
    def now() -> _Now:
        return _Clock._Now()


def _pose(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    orientation: Quaternion | None = None,
) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = "base_footprint"
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z
    message.pose.orientation = orientation or Quaternion(w=1.0)
    return message


def test_clamp_vector_limits_magnitude_without_changing_direction() -> None:
    assert _clamp_vector((3.0, 4.0, 0.0), 2.0) == pytest.approx(
        (1.2, 1.6, 0.0)
    )


def test_non_positive_vector_limit_disables_bridge_clamping() -> None:
    assert _clamp_vector((3.0, 4.0, 0.0), 0.0) == (3.0, 4.0, 0.0)


def test_non_finite_quaternion_is_rejected() -> None:
    assert _normalize_quaternion(Quaternion(y=math.inf, w=1.0)) is False


def test_pose_feedback_uses_vector_deadband() -> None:
    assert _pose_feedback((0.0005, 0.0005, 0.0005), 5.0, 0.001) == (
        0.0,
        0.0,
        0.0,
    )
    assert _pose_feedback((0.001, 0.001, 0.0), 5.0, 0.001) == pytest.approx(
        (0.005, 0.005, 0.0)
    )


def test_orientation_error_uses_shortest_quaternion_path() -> None:
    half_angle = math.pi / 4.0
    target = Quaternion(z=math.sin(half_angle), w=-math.cos(half_angle))
    current = Quaternion(w=1.0)

    error = ServoPoseBridge._orientation_error(target, current)

    assert error == pytest.approx((0.0, 0.0, -math.pi / 2.0))


def test_target_velocity_is_filtered_and_resets_after_gap() -> None:
    bridge = object.__new__(ServoPoseBridge)
    bridge.previous_target = None
    bridge.previous_target_received_sec = 0.0
    bridge.target_linear_velocity = (0.0, 0.0, 0.0)
    bridge.target_angular_velocity = (0.0, 0.0, 0.0)
    bridge.feedforward_reset_gap_sec = 0.12
    bridge.feedforward_filter_alpha = 1.0
    bridge.max_linear_velocity_mps = 0.5
    bridge.max_angular_velocity_radps = 1.5
    bridge.linear_feedforward_stop_velocity_mps = 0.005
    bridge.angular_feedforward_stop_velocity_radps = 0.02

    bridge._update_target_velocity(_pose(), 1.0)
    bridge._update_target_velocity(_pose(x=0.01), 1.02)
    assert bridge.target_linear_velocity == pytest.approx((0.5, 0.0, 0.0))

    bridge._update_target_velocity(_pose(x=0.50), 1.20)
    assert bridge.target_linear_velocity == (0.0, 0.0, 0.0)
    assert bridge.target_angular_velocity == (0.0, 0.0, 0.0)


def test_target_velocity_stops_immediately_at_stationary_target() -> None:
    bridge = object.__new__(ServoPoseBridge)
    bridge.previous_target = _pose(x=0.01)
    bridge.previous_target_received_sec = 1.0
    bridge.target_linear_velocity = (0.2, 0.0, 0.0)
    bridge.target_angular_velocity = (0.3, 0.0, 0.0)
    bridge.feedforward_reset_gap_sec = 0.12
    bridge.feedforward_filter_alpha = 0.35
    bridge.max_linear_velocity_mps = 0.5
    bridge.max_angular_velocity_radps = 1.5
    bridge.linear_feedforward_stop_velocity_mps = 0.005
    bridge.angular_feedforward_stop_velocity_radps = 0.02

    bridge._update_target_velocity(_pose(x=0.01), 1.02)

    assert bridge.target_linear_velocity == (0.0, 0.0, 0.0)
    assert bridge.target_angular_velocity == (0.0, 0.0, 0.0)


def test_deadman_release_clears_target_and_publishes_halt_commands() -> None:
    bridge = object.__new__(ServoPoseBridge)
    bridge.pose_commands_active = True
    bridge.latest_target = _pose(x=0.3)
    bridge.latest_target_received_sec = 1.0
    bridge.previous_target = _pose(x=0.2)
    bridge.previous_target_received_sec = 0.9
    bridge.target_linear_velocity = (0.5, 0.0, 0.0)
    bridge.target_angular_velocity = (0.0, 0.0, 1.0)
    bridge.halt_command_count = 4
    bridge.planning_frame = "base_footprint"
    bridge.twist_publisher = _Publisher()
    bridge.get_clock = lambda: _Clock()
    bridge.get_logger = lambda: type(
        "Logger",
        (),
        {"info": lambda self, message: None},
    )()

    bridge._on_pose_active(Bool(data=False))

    assert bridge.pose_commands_active is False
    assert bridge.latest_target is None
    assert bridge.previous_target is None
    assert bridge.target_linear_velocity == (0.0, 0.0, 0.0)
    assert bridge.target_angular_velocity == (0.0, 0.0, 0.0)
    assert len(bridge.twist_publisher.messages) == 4
    assert all(
        message.header.frame_id == "base_footprint"
        and message.twist.linear.x == 0.0
        and message.twist.linear.y == 0.0
        and message.twist.linear.z == 0.0
        and message.twist.angular.x == 0.0
        and message.twist.angular.y == 0.0
        and message.twist.angular.z == 0.0
        for message in bridge.twist_publisher.messages
    )


def test_pose_commands_are_ignored_while_deadman_is_inactive() -> None:
    bridge = object.__new__(ServoPoseBridge)
    bridge.pose_commands_active = False
    bridge.latest_target = None

    bridge._on_pose_command(_pose(x=0.3))

    assert bridge.latest_target is None
