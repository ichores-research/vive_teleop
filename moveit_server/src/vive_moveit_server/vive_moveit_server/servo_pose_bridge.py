import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.task import Future
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def _normalize_quaternion(quaternion: Quaternion) -> bool:
    norm_squared = (
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm_squared < 1e-12:
        return False

    norm = math.sqrt(norm_squared)
    quaternion.x /= norm
    quaternion.y /= norm
    quaternion.z /= norm
    quaternion.w /= norm
    return True


def _inverse_quaternion(quaternion: Quaternion) -> Quaternion:
    inverse = Quaternion()
    inverse.x = -quaternion.x
    inverse.y = -quaternion.y
    inverse.z = -quaternion.z
    inverse.w = quaternion.w
    return inverse


def _multiply_quaternions(left: Quaternion, right: Quaternion) -> Quaternion:
    result = Quaternion()
    result.x = (
        left.w * right.x
        + left.x * right.w
        + left.y * right.z
        - left.z * right.y
    )
    result.y = (
        left.w * right.y
        - left.x * right.z
        + left.y * right.w
        + left.z * right.x
    )
    result.z = (
        left.w * right.z
        + left.x * right.y
        - left.y * right.x
        + left.z * right.w
    )
    result.w = (
        left.w * right.w
        - left.x * right.x
        - left.y * right.y
        - left.z * right.z
    )
    _normalize_quaternion(result)
    return result


def _clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


class ServoPoseBridge(Node):
    """Convert absolute end-effector poses into Humble Servo twist commands."""

    def __init__(self) -> None:
        super().__init__("servo_pose_bridge")

        self.pose_command_topic = self.declare_parameter(
            "pose_command_topic",
            "/servo_node/pose_target_cmds",
        ).value
        self.twist_command_topic = self.declare_parameter(
            "twist_command_topic",
            "/servo_node/delta_twist_cmds",
        ).value
        self.start_servo_service = self.declare_parameter(
            "start_servo_service",
            "/servo_node/start_servo",
        ).value
        self.planning_frame = self.declare_parameter(
            "planning_frame",
            "base_footprint",
        ).value
        self.end_effector_frame = self.declare_parameter(
            "end_effector_frame",
            "arm_tool_link",
        ).value

        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 50.0).value
        )
        self.target_timeout_sec = float(
            self.declare_parameter("target_timeout_sec", 0.12).value
        )
        self.linear_gain = float(
            self.declare_parameter("linear_gain", 1.5).value
        )
        self.angular_gain = float(
            self.declare_parameter("angular_gain", 1.0).value
        )
        self.max_linear_velocity_mps = float(
            self.declare_parameter("max_linear_velocity_mps", 0.20).value
        )
        self.max_angular_velocity_radps = float(
            self.declare_parameter("max_angular_velocity_radps", 0.60).value
        )
        self.position_deadband_m = float(
            self.declare_parameter("position_deadband_m", 0.002).value
        )
        self.orientation_deadband_rad = float(
            self.declare_parameter("orientation_deadband_rad", 0.01).value
        )

        self.latest_target: Optional[PoseStamped] = None
        self.latest_target_received_sec = 0.0
        self.servo_started = False
        self.start_request_in_flight = False
        self.last_start_attempt_sec = 0.0
        self.last_log_times: dict[str, float] = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.twist_publisher = self.create_publisher(
            TwistStamped,
            self.twist_command_topic,
            qos_profile_system_default,
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_command_topic,
            self._on_pose_command,
            qos_profile_system_default,
        )
        self.start_client = self.create_client(
            Trigger,
            self.start_servo_service,
        )

        timer_period_sec = 1.0 / max(1.0, self.publish_rate_hz)
        self.create_timer(timer_period_sec, self._update)
        self.get_logger().info(
            f"Bridging Servo pose commands on '{self.pose_command_topic}' "
            f"to twists on '{self.twist_command_topic}'"
        )

    def _on_pose_command(self, message: PoseStamped) -> None:
        if message.header.frame_id != self.planning_frame:
            self._warn_throttled(
                "pose_frame",
                f"Ignoring Servo pose in frame '{message.header.frame_id}'; "
                f"expected '{self.planning_frame}'",
                2.0,
            )
            return

        target = PoseStamped()
        target.header = message.header
        target.pose = message.pose
        if not _normalize_quaternion(target.pose.orientation):
            self._warn_throttled(
                "pose_quaternion",
                "Ignoring Servo pose with invalid orientation",
                2.0,
            )
            return

        self.latest_target = target
        self.latest_target_received_sec = self._now_sec()

    def _update(self) -> None:
        if not self._ensure_servo_started():
            return
        if self.latest_target is None:
            return
        if (
            self.target_timeout_sec > 0.0
            and self._now_sec() - self.latest_target_received_sec
            > self.target_timeout_sec
        ):
            self.latest_target = None
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.end_effector_frame,
                Time(),
            )
        except TransformException as error:
            self._warn_throttled(
                "wrist_tf",
                "Waiting for wrist transform "
                f"'{self.planning_frame}' -> '{self.end_effector_frame}': "
                f"{error}",
                2.0,
            )
            return

        current_orientation = transform.transform.rotation
        if not _normalize_quaternion(current_orientation):
            self._warn_throttled(
                "wrist_quaternion",
                "Ignoring wrist TF with invalid orientation",
                2.0,
            )
            return

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self.planning_frame
        command.twist.linear.x = self._linear_command(
            self.latest_target.pose.position.x
            - transform.transform.translation.x
        )
        command.twist.linear.y = self._linear_command(
            self.latest_target.pose.position.y
            - transform.transform.translation.y
        )
        command.twist.linear.z = self._linear_command(
            self.latest_target.pose.position.z
            - transform.transform.translation.z
        )

        angular_error = self._orientation_error(
            self.latest_target.pose.orientation,
            current_orientation,
        )
        command.twist.angular.x = self._angular_command(angular_error[0])
        command.twist.angular.y = self._angular_command(angular_error[1])
        command.twist.angular.z = self._angular_command(angular_error[2])
        self.twist_publisher.publish(command)

    def _ensure_servo_started(self) -> bool:
        if self.servo_started:
            return True
        if self.start_request_in_flight:
            return False

        now_sec = self._now_sec()
        if now_sec - self.last_start_attempt_sec < 1.0:
            return False
        self.last_start_attempt_sec = now_sec

        if not self.start_client.wait_for_service(timeout_sec=0.0):
            self._warn_throttled(
                "start_service",
                f"Waiting for MoveIt Servo service '{self.start_servo_service}'",
                5.0,
            )
            return False

        self.start_request_in_flight = True
        future = self.start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_servo_started)
        return False

    def _on_servo_started(self, future: Future) -> None:
        self.start_request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(f"MoveIt Servo start request failed: {error}")
            return

        if not response.success:
            self._warn_throttled(
                "start_rejected",
                f"MoveIt Servo did not start: {response.message}",
                2.0,
            )
            return

        self.servo_started = True
        self.get_logger().info("MoveIt Servo started")

    def _linear_command(self, error: float) -> float:
        if abs(error) < self.position_deadband_m:
            return 0.0
        return _clamp(
            error * self.linear_gain,
            self.max_linear_velocity_mps,
        )

    def _angular_command(self, error: float) -> float:
        if abs(error) < self.orientation_deadband_rad:
            return 0.0
        return _clamp(
            error * self.angular_gain,
            self.max_angular_velocity_radps,
        )

    @staticmethod
    def _orientation_error(
        target: Quaternion,
        current: Quaternion,
    ) -> tuple[float, float, float]:
        error = _multiply_quaternions(
            target,
            _inverse_quaternion(current),
        )
        if error.w < 0.0:
            error.x = -error.x
            error.y = -error.y
            error.z = -error.z
            error.w = -error.w

        vector_norm = math.sqrt(
            (error.x * error.x)
            + (error.y * error.y)
            + (error.z * error.z)
        )
        if vector_norm < 1e-9:
            return (0.0, 0.0, 0.0)

        angle = 2.0 * math.atan2(vector_norm, max(0.0, error.w))
        scale = angle / vector_norm
        return (
            error.x * scale,
            error.y * scale,
            error.z * scale,
        )

    def _warn_throttled(
        self,
        key: str,
        message: str,
        period_sec: float,
    ) -> None:
        now_sec = self._now_sec()
        if now_sec - self.last_log_times.get(key, 0.0) < period_sec:
            return
        self.last_log_times[key] = now_sec
        self.get_logger().warn(message)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
