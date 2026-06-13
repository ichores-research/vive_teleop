import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .pose_utils import (
    clamp,
    limit_vector_change,
    limit_vector_norm,
    normalize_quaternion,
    orientation_error_vector,
)


SERVO_STATUS_DESCRIPTIONS = {
    -1: "invalid status",
    0: "no warnings",
    1: "approaching a singularity; decelerating",
    2: "too close to a singularity; halted",
    3: "approaching a collision; decelerating",
    4: "collision detected; halted",
    5: "close to a joint position or velocity bound; halted",
    6: "leaving a singularity; decelerating",
}


class ServoController:
    """Converts robot-frame wrist pose targets into MoveIt Servo twists."""

    def __init__(
        self,
        node: Node,
        end_effector_link: str,
        pose_reference_frame: str,
        position_deadband_m: float,
        orientation_deadband_rad: float,
    ) -> None:
        self._node = node
        self._end_effector_link = end_effector_link
        self._pose_reference_frame = pose_reference_frame
        self._position_deadband_m = position_deadband_m
        self._orientation_deadband_rad = orientation_deadband_rad

        self.cartesian_command_topic = node.declare_parameter(
            "servo_cartesian_command_topic",
            "/servo_node/delta_twist_cmds",
        ).value
        self.status_topic = node.declare_parameter(
            "servo_status_topic",
            "/servo_node/status",
        ).value
        self.start_service_name = node.declare_parameter(
            "servo_start_service_name",
            "/servo_node/start_servo",
        ).value
        self.publish_rate_hz = float(
            node.declare_parameter("servo_publish_rate_hz", 50.0).value
        )
        self._linear_gain = float(
            node.declare_parameter("servo_linear_gain", 6.0).value
        )
        self._angular_gain = float(
            node.declare_parameter("servo_angular_gain", 5.0).value
        )
        self._max_linear_velocity_mps = float(
            node.declare_parameter(
                "servo_max_linear_velocity_mps",
                0.45,
            ).value
        )
        self._max_angular_velocity_radps = float(
            node.declare_parameter(
                "servo_max_angular_velocity_radps",
                1.8,
            ).value
        )
        self._max_linear_acceleration_mps2 = float(
            node.declare_parameter(
                "servo_max_linear_acceleration_mps2",
                2.5,
            ).value
        )
        self._max_angular_acceleration_radps2 = float(
            node.declare_parameter(
                "servo_max_angular_acceleration_radps2",
                8.0,
            ).value
        )

        self._twist_publisher = node.create_publisher(
            TwistStamped,
            self.cartesian_command_topic,
            10,
        )
        self._start_client = node.create_client(
            Trigger,
            self.start_service_name,
        )
        node.create_subscription(
            Int8,
            self.status_topic,
            self._on_status,
            10,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._last_command_sec = 0.0
        self._last_linear_velocity = (0.0, 0.0, 0.0)
        self._last_angular_velocity = (0.0, 0.0, 0.0)
        self._last_status: Optional[int] = None
        self._hold_active = False
        self._start_in_flight = False
        self._started = False
        self._last_log_times: dict[str, float] = {}

    def start_hold(self) -> None:
        self._hold_active = True
        self._last_command_sec = self._now_sec()
        self._last_linear_velocity = (0.0, 0.0, 0.0)
        self._last_angular_velocity = (0.0, 0.0, 0.0)

    def stop_hold(self) -> None:
        self._publish_twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self._hold_active = False
        self._last_command_sec = 0.0
        self._last_linear_velocity = (0.0, 0.0, 0.0)
        self._last_angular_velocity = (0.0, 0.0, 0.0)

    def ensure_started(self) -> bool:
        if self._started:
            return True
        if self._start_in_flight:
            return False
        if not self._start_client.wait_for_service(timeout_sec=0.0):
            self._warn_throttled(
                "servo_start_wait",
                f"Waiting for MoveIt Servo service '{self.start_service_name}'",
                5.0,
            )
            return False

        self._start_in_flight = True
        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_started)
        return False

    def lookup_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._pose_reference_frame,
                self._end_effector_link,
                Time(),
            )
        except TransformException as error:
            self._warn_throttled(
                "servo_tf_wait",
                f"Waiting for TF {self._pose_reference_frame} -> "
                f"{self._end_effector_link}: {error}",
                2.0,
            )
            return None

        current_pose = PoseStamped()
        current_pose.header = transform.header
        current_pose.pose.position.x = transform.transform.translation.x
        current_pose.pose.position.y = transform.transform.translation.y
        current_pose.pose.position.z = transform.transform.translation.z
        current_pose.pose.orientation = transform.transform.rotation
        if not normalize_quaternion(current_pose.pose.orientation):
            self._warn_throttled(
                "servo_tf_quaternion",
                "TF returned an invalid wrist orientation",
                2.0,
            )
            return None
        return current_pose

    def publish_target(
        self,
        target: PoseStamped,
        current_pose: PoseStamped,
    ) -> bool:
        if not self.ensure_started():
            return False

        linear_error = (
            target.pose.position.x - current_pose.pose.position.x,
            target.pose.position.y - current_pose.pose.position.y,
            target.pose.position.z - current_pose.pose.position.z,
        )
        angular_error = orientation_error_vector(
            current_pose.pose.orientation,
            target.pose.orientation,
        )

        if math.sqrt(sum(value * value for value in linear_error)) < max(
            0.0,
            self._position_deadband_m,
        ):
            linear_error = (0.0, 0.0, 0.0)
        if math.sqrt(sum(value * value for value in angular_error)) < max(
            0.0,
            self._orientation_deadband_rad,
        ):
            angular_error = (0.0, 0.0, 0.0)

        desired_linear_velocity = limit_vector_norm(
            tuple(self._linear_gain * value for value in linear_error),
            max(0.0, self._max_linear_velocity_mps),
        )
        desired_angular_velocity = limit_vector_norm(
            tuple(self._angular_gain * value for value in angular_error),
            max(0.0, self._max_angular_velocity_radps),
        )

        now_sec = self._now_sec()
        nominal_period_sec = 1.0 / max(1.0, self.publish_rate_hz)
        elapsed_sec = nominal_period_sec
        if self._last_command_sec > 0.0:
            elapsed_sec = clamp(
                now_sec - self._last_command_sec,
                0.0,
                nominal_period_sec * 2.0,
            )
        linear_velocity = limit_vector_change(
            self._last_linear_velocity,
            desired_linear_velocity,
            max(0.0, self._max_linear_acceleration_mps2) * elapsed_sec,
        )
        angular_velocity = limit_vector_change(
            self._last_angular_velocity,
            desired_angular_velocity,
            max(0.0, self._max_angular_acceleration_radps2) * elapsed_sec,
        )
        self._publish_twist(linear_velocity, angular_velocity)
        self._last_command_sec = now_sec
        self._last_linear_velocity = linear_velocity
        self._last_angular_velocity = angular_velocity
        return True

    def _on_started(self, future: Future) -> None:
        self._start_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self._node.get_logger().warn(
                f"MoveIt Servo start request failed: {error}"
            )
            return

        if not response.success:
            self._node.get_logger().warn(
                f"MoveIt Servo refused to start: {response.message}"
            )
            return

        self._started = True
        self._node.get_logger().info("MoveIt Servo started")

    def _on_status(self, message: Int8) -> None:
        status = int(message.data)
        if status == self._last_status:
            return

        previous_status = self._last_status
        self._last_status = status
        description = SERVO_STATUS_DESCRIPTIONS.get(status, "unknown status")
        if status == 0:
            if previous_status not in (None, 0):
                self._node.get_logger().info(
                    "MoveIt Servo recovered: no warnings"
                )
            return

        self._node.get_logger().warning(
            f"MoveIt Servo status {status}: {description}"
        )

    def _publish_twist(
        self,
        linear: tuple[float, float, float],
        angular: tuple[float, float, float],
    ) -> None:
        command = TwistStamped()
        command.header.stamp = self._node.get_clock().now().to_msg()
        command.header.frame_id = self._pose_reference_frame
        command.twist.linear.x = linear[0]
        command.twist.linear.y = linear[1]
        command.twist.linear.z = linear[2]
        command.twist.angular.x = angular[0]
        command.twist.angular.y = angular[1]
        command.twist.angular.z = angular[2]
        self._twist_publisher.publish(command)

    def _warn_throttled(
        self,
        key: str,
        message: str,
        period_sec: float,
    ) -> None:
        now_sec = self._now_sec()
        last_sec = self._last_log_times.get(key)
        if last_sec is not None and now_sec - last_sec < period_sec:
            return

        self._last_log_times[key] = now_sec
        self._node.get_logger().warn(message)

    def _now_sec(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9
