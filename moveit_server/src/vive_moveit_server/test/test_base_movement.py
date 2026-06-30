import pytest
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

from vive_moveit_server.base_movement import (
    BaseMovementMixin,
    _approach_velocity,
)


class _MessageSink:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _Now:
    def __init__(self, seconds: float) -> None:
        self.nanoseconds = int(seconds * 1e9)


class _Clock:
    def __init__(self) -> None:
        self.seconds = 1.0

    def now(self) -> _Now:
        return _Now(self.seconds)


class _BaseHarness(BaseMovementMixin):
    def __init__(self) -> None:
        self.clock = _Clock()
        self.base_velocity_publisher = _MessageSink()
        self.base_command_frame = "base_footprint"
        self.base_input_timeout_sec = 0.15
        self.base_publish_rate_hz = 30.0
        self.base_max_linear_velocity_mps = 0.25
        self.base_max_angular_velocity_radps = 0.6
        self.base_max_linear_acceleration_mps2 = 0.5
        self.base_max_angular_acceleration_radps2 = 1.2
        self.base_max_linear_deceleration_mps2 = 1.0
        self.base_max_angular_deceleration_radps2 = 2.4
        self.base_halt_command_count = 3
        self.base_active_input = False
        self.base_command_enabled = False
        self.pending_base_command = None
        self.base_enabled_received_sec = 0.0
        self.last_base_command_received_sec = 0.0
        self.last_base_update_sec = 0.0
        self.last_base_output_linear = 0.0
        self.last_base_output_angular = 0.0
        self.base_halt_commands_remaining = 0
        self.warnings = []

    def get_clock(self) -> _Clock:
        return self.clock

    def _warn_throttled(
        self,
        _key: str,
        message: str,
        _period_sec: float,
    ) -> None:
        self.warnings.append(message)


def _command(linear_x: float, angular_z: float) -> TwistStamped:
    message = TwistStamped()
    message.header.frame_id = "base_footprint"
    message.twist.linear.x = linear_x
    message.twist.angular.z = angular_z
    return message


def test_approach_velocity_uses_acceleration_and_deceleration_limits() -> None:
    assert _approach_velocity(
        0.0,
        1.0,
        acceleration=0.5,
        deceleration=1.0,
        dt=0.1,
    ) == pytest.approx(0.05)
    assert _approach_velocity(
        0.5,
        0.0,
        acceleration=0.5,
        deceleration=1.0,
        dt=0.1,
    ) == pytest.approx(0.4)


def test_active_base_command_is_clamped_smoothed_and_published() -> None:
    guard = _BaseHarness()
    guard._on_base_active(Bool(data=True))
    guard._on_base_command(_command(2.0, -2.0))

    guard._maybe_publish_base_command()

    output = guard.base_velocity_publisher.messages[-1]
    assert output.linear.x == pytest.approx(0.5 / 30.0)
    assert output.angular.z == pytest.approx(-1.2 / 30.0)


def test_deadman_release_publishes_an_immediate_zero() -> None:
    guard = _BaseHarness()
    guard._on_base_active(Bool(data=True))
    guard._on_base_command(_command(0.2, 0.4))
    guard._maybe_publish_base_command()

    guard._on_base_active(Bool(data=False))

    output = guard.base_velocity_publisher.messages[-1]
    assert output.linear.x == 0.0
    assert output.angular.z == 0.0
    assert guard.base_halt_commands_remaining == 2


def test_timeout_requires_a_new_physical_button_edge() -> None:
    guard = _BaseHarness()
    guard._on_base_active(Bool(data=True))
    guard._on_base_command(_command(0.2, 0.4))
    guard.clock.seconds += 0.2

    guard._maybe_publish_base_command()

    assert guard.base_command_enabled is False
    assert guard.base_velocity_publisher.messages[-1].linear.x == 0.0

    guard._on_base_active(Bool(data=True))
    guard._on_base_command(_command(0.2, 0.4))
    assert guard.base_command_enabled is False
    assert guard.pending_base_command is None

    guard._on_base_active(Bool(data=False))
    guard._on_base_active(Bool(data=True))
    assert guard.base_command_enabled is True
