import math

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool


def _approach_velocity(
    current: float,
    target: float,
    *,
    acceleration: float,
    deceleration: float,
    dt: float,
) -> float:
    if current == target:
        return target

    slowing = (current * target < 0.0) or abs(target) < abs(current)
    rate = deceleration if slowing else acceleration
    max_delta = max(0.0, rate) * max(0.0, dt)
    difference = target - current
    if abs(difference) <= max_delta:
        return target
    return current + math.copysign(max_delta, difference)


class BaseMovementMixin:
    """Guard and smooth base intent before publishing TIAGo velocity commands."""

    def _on_base_active(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self.base_active_input:
            self.base_command_enabled = True
            self.pending_base_command = None
            self.base_enabled_received_sec = self._base_now_sec()
        elif not requested and self.base_active_input:
            self.base_command_enabled = False
            self.pending_base_command = None
            self._halt_base_immediately()

        self.base_active_input = requested

    def _on_base_command(self, message: TwistStamped) -> None:
        if not self.base_command_enabled:
            return

        if message.header.frame_id not in ("", self.base_command_frame):
            self._warn_throttled(
                "invalid_base_command_frame",
                "Ignoring base command in frame "
                f"'{message.header.frame_id}'; expected "
                f"'{self.base_command_frame}'",
                2.0,
            )
            return

        linear_x = float(message.twist.linear.x)
        angular_z = float(message.twist.angular.z)
        if not all(math.isfinite(value) for value in (linear_x, angular_z)):
            self._warn_throttled(
                "non_finite_base_command",
                "Ignoring non-finite base velocity command",
                2.0,
            )
            return

        max_linear = max(0.0, self.base_max_linear_velocity_mps)
        max_angular = max(0.0, self.base_max_angular_velocity_radps)
        self.pending_base_command = (
            max(-max_linear, min(max_linear, linear_x)),
            max(-max_angular, min(max_angular, angular_z)),
        )
        self.last_base_command_received_sec = self._base_now_sec()

    def _maybe_publish_base_command(self) -> None:
        now_sec = self._base_now_sec()
        target = self._fresh_base_target(now_sec)
        if target is None:
            if self.base_command_enabled:
                self.base_command_enabled = False
                self.pending_base_command = None
                self._warn_throttled(
                    "base_command_timeout",
                    "Base command timed out; release and press the trackpad "
                    "again before driving",
                    2.0,
                )
                self._halt_base_immediately()
            elif self.base_halt_commands_remaining > 0:
                self._publish_base_velocity(0.0, 0.0)
                self.base_halt_commands_remaining -= 1
            return

        if self.last_base_update_sec <= 0.0:
            dt = 1.0 / max(1.0, self.base_publish_rate_hz)
        else:
            dt = max(0.0, min(0.1, now_sec - self.last_base_update_sec))
        self.last_base_update_sec = now_sec

        target_linear, target_angular = target
        next_linear = _approach_velocity(
            self.last_base_output_linear,
            target_linear,
            acceleration=self.base_max_linear_acceleration_mps2,
            deceleration=self.base_max_linear_deceleration_mps2,
            dt=dt,
        )
        next_angular = _approach_velocity(
            self.last_base_output_angular,
            target_angular,
            acceleration=self.base_max_angular_acceleration_radps2,
            deceleration=self.base_max_angular_deceleration_radps2,
            dt=dt,
        )
        self._publish_base_velocity(next_linear, next_angular)

    def _fresh_base_target(
        self,
        now_sec: float,
    ) -> tuple[float, float] | None:
        if not self.base_command_enabled:
            return None

        timeout = max(0.02, self.base_input_timeout_sec)
        if self.pending_base_command is None:
            if now_sec - self.base_enabled_received_sec <= timeout:
                return 0.0, 0.0
            return None
        if now_sec - self.last_base_command_received_sec > timeout:
            return None
        return self.pending_base_command

    def _halt_base_immediately(self) -> None:
        self.last_base_output_linear = 0.0
        self.last_base_output_angular = 0.0
        self.last_base_update_sec = self._base_now_sec()
        self._publish_base_velocity(0.0, 0.0)
        self.base_halt_commands_remaining = max(
            0,
            int(self.base_halt_command_count) - 1,
        )

    def _publish_base_velocity(
        self,
        linear_x: float,
        angular_z: float,
    ) -> None:
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.base_velocity_publisher.publish(command)
        self.last_base_output_linear = linear_x
        self.last_base_output_angular = angular_z

    def _base_now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9
