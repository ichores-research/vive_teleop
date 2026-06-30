import math

import pytest
from builtin_interfaces.msg import Time

from image_listener.input_publisher import WebRtcInputPublisher


class _MessageSink:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _Logger:
    def __init__(self) -> None:
        self.warnings = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class _Now:
    nanoseconds = 1_000_000_000

    def to_msg(self) -> Time:
        return Time(sec=1, nanosec=0)


class _Clock:
    def now(self) -> _Now:
        return _Now()


def _publisher_with_sink() -> tuple[WebRtcInputPublisher, _MessageSink]:
    publisher = object.__new__(WebRtcInputPublisher)
    sink = _MessageSink()
    publisher._hand_target_active_publisher = sink
    return publisher, sink


def _base_publisher_with_sinks() -> tuple[
    WebRtcInputPublisher,
    _MessageSink,
    _MessageSink,
]:
    publisher = object.__new__(WebRtcInputPublisher)
    active_sink = _MessageSink()
    command_sink = _MessageSink()
    publisher._base_active_publisher = active_sink
    publisher._base_command_publisher = command_sink
    publisher._base_frame = "base_footprint"
    publisher._base_joystick_deadzone = 0.15
    publisher._base_max_linear_velocity_mps = 0.25
    publisher._base_max_angular_velocity_radps = 0.6
    publisher.get_clock = lambda: _Clock()
    return publisher, active_sink, command_sink


def test_explicit_deadman_release_is_published() -> None:
    publisher, sink = _publisher_with_sink()

    publisher._publish_hand_target_active(
        {
            "wristAvailable": True,
            "wristCommandEnabled": False,
        }
    )

    assert len(sink.messages) == 1
    assert sink.messages[0].data is False


def test_legacy_wrist_payload_defaults_to_active() -> None:
    publisher, sink = _publisher_with_sink()

    publisher._publish_hand_target_active({"wristAvailable": True})

    assert len(sink.messages) == 1
    assert sink.messages[0].data is True


def test_missing_wrist_does_not_change_deadman_state() -> None:
    publisher, sink = _publisher_with_sink()

    publisher._publish_hand_target_active({"wristAvailable": False})

    assert sink.messages == []


def test_non_boolean_deadman_values_fail_closed() -> None:
    publisher, sink = _publisher_with_sink()
    logger = _Logger()
    publisher.get_logger = lambda: logger

    for value in ("false", None, 1):
        publisher._publish_hand_target_active(
            {
                "wristAvailable": True,
                "wristCommandEnabled": value,
            }
        )

    assert len(sink.messages) == 3
    assert all(message.data is False for message in sink.messages)
    assert logger.warnings == [
        "Ignoring non-boolean wrist command gate",
        "Ignoring non-boolean wrist command gate",
        "Ignoring non-boolean wrist command gate",
    ]


def test_string_deadman_value_does_not_publish_a_wrist_target() -> None:
    publisher = object.__new__(WebRtcInputPublisher)
    sink = _MessageSink()
    publisher._hand_target_publisher = sink

    publisher._publish_hand_target(
        {
            "wristAvailable": True,
            "wristCommandEnabled": "true",
        }
    )

    assert sink.messages == []


def test_oversized_input_is_rejected_before_parsing() -> None:
    publisher = object.__new__(WebRtcInputPublisher)
    logger = _Logger()
    publisher.get_logger = lambda: logger

    publisher.publish_input("x" * ((64 * 1024) + 1))

    assert logger.warnings == ["Ignoring WebRTC input larger than 64 KiB"]


def test_non_finite_position_is_rejected_before_publication() -> None:
    publisher = object.__new__(WebRtcInputPublisher)
    logger = _Logger()
    publisher.get_logger = lambda: logger

    message = publisher._extract_pose(
        {
            "px": math.inf,
            "py": 0.0,
            "pz": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        },
        position_prefix="p",
        quaternion_prefix="q",
        frame_id="test",
        warning_context="test pose",
    )

    assert message is None
    assert logger.warnings == ["Ignoring non-finite test pose position"]


def test_non_finite_quaternion_is_rejected() -> None:
    publisher = object.__new__(WebRtcInputPublisher)
    logger = _Logger()
    publisher.get_logger = lambda: logger

    quaternion = publisher._extract_quaternion(
        {"qx": math.nan, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        {"x": "qx", "y": "qy", "z": "qz", "w": "qw"},
        "test pose",
    )

    assert quaternion is None
    assert logger.warnings == ["Ignoring non-finite test pose quaternion"]


def test_huge_finite_quaternion_is_normalized_without_overflow() -> None:
    publisher = object.__new__(WebRtcInputPublisher)
    logger = _Logger()
    publisher.get_logger = lambda: logger

    quaternion = publisher._extract_quaternion(
        {"qx": 1e308, "qy": 1e308, "qz": 1e308, "qw": 1e308},
        {"x": "qx", "y": "qy", "z": "qz", "w": "qw"},
        "test pose",
    )

    assert quaternion is not None
    assert all(math.isfinite(value) for value in quaternion.values())
    assert math.sqrt(sum(value * value for value in quaternion.values())) == 1.0


def test_pressed_joystick_up_commands_forward_motion() -> None:
    publisher, active_sink, command_sink = _base_publisher_with_sinks()

    publisher._publish_base_input(
        {
            "wristAvailable": True,
            "joystickPrimaryButton": True,
            "joystickAxisX": 0.0,
            "joystickAxisY": 1.0,
        }
    )

    assert active_sink.messages[-1].data is True
    command = command_sink.messages[-1]
    assert command.twist.linear.x == pytest.approx(0.25)
    assert command.twist.angular.z == pytest.approx(0.0)


def test_pressed_joystick_left_commands_positive_yaw() -> None:
    publisher, _active_sink, command_sink = _base_publisher_with_sinks()

    publisher._publish_base_input(
        {
            "wristAvailable": True,
            "joystickPrimaryButton": True,
            "joystickAxisX": -1.0,
            "joystickAxisY": 0.0,
        }
    )

    command = command_sink.messages[-1]
    assert command.twist.linear.x == pytest.approx(0.0)
    assert command.twist.angular.z == pytest.approx(0.6)


def test_diagonal_joystick_continuously_mixes_forward_and_turning() -> None:
    publisher, _active_sink, command_sink = _base_publisher_with_sinks()
    diagonal = math.sqrt(0.5)

    publisher._publish_base_input(
        {
            "wristAvailable": True,
            "joystickPrimaryButton": True,
            "joystickAxisX": -diagonal,
            "joystickAxisY": diagonal,
        }
    )

    command = command_sink.messages[-1]
    assert command.twist.linear.x == pytest.approx(0.25 * diagonal)
    assert command.twist.angular.z == pytest.approx(0.6 * diagonal)


def test_base_joystick_uses_a_radial_deadzone_without_sector_quantization() -> None:
    linear_x, angular_z = WebRtcInputPublisher._joystick_to_base_velocity(
        -0.2,
        0.2,
        deadzone=0.15,
        max_linear_velocity_mps=0.25,
        max_angular_velocity_radps=0.6,
    )

    assert 0.0 < linear_x < 0.25
    assert 0.0 < angular_z < 0.6
    assert angular_z / linear_x == pytest.approx(0.6 / 0.25)


def test_releasing_joystick_click_publishes_inactive_without_command() -> None:
    publisher, active_sink, command_sink = _base_publisher_with_sinks()

    publisher._publish_base_input(
        {
            "wristAvailable": True,
            "joystickPrimaryButton": False,
            "joystickAxisX": -1.0,
            "joystickAxisY": 1.0,
        }
    )

    assert active_sink.messages[-1].data is False
    assert command_sink.messages == []
