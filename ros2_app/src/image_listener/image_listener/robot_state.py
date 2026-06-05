import math
import threading
from typing import Any

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


def _head_command_quaternion(
    pan: float,
    tilt: float,
    pan_sign: float,
    tilt_sign: float,
) -> dict[str, float]:
    input_pan = pan / pan_sign if abs(pan_sign) > 1e-9 else pan
    input_tilt = tilt / tilt_sign if abs(tilt_sign) > 1e-9 else tilt

    half_pan = input_pan * 0.5
    half_tilt = input_tilt * 0.5
    cos_pan = math.cos(half_pan)
    sin_pan = math.sin(half_pan)
    cos_tilt = math.cos(half_tilt)
    sin_tilt = math.sin(half_tilt)

    # Matches vive_moveit_server's Unity Y-yaw/X-pitch extraction.
    return {
        "x": cos_pan * sin_tilt,
        "y": sin_pan * cos_tilt,
        "z": -sin_pan * sin_tilt,
        "w": cos_pan * cos_tilt,
    }


def _transform_pose(transform: Any) -> dict[str, dict[str, float]]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        "position": {
            "x": float(translation.x),
            "y": float(translation.y),
            "z": float(translation.z),
        },
        "orientation": {
            "x": float(rotation.x),
            "y": float(rotation.y),
            "z": float(rotation.z),
            "w": float(rotation.w),
        },
    }


class RobotInputState(Node):
    def __init__(self) -> None:
        super().__init__("robot_input_state")

        self._reference_frame = self.declare_parameter(
            "robot_state_reference_frame",
            "base_footprint",
        ).value
        self._hand_frame = self.declare_parameter(
            "robot_hand_frame",
            "arm_tool_link",
        ).value
        self._head_frame = self.declare_parameter(
            "robot_head_frame",
            "head_front_camera_link",
        ).value
        self._head_pan_joint = self.declare_parameter(
            "robot_head_pan_joint",
            "head_1_joint",
        ).value
        self._head_tilt_joint = self.declare_parameter(
            "robot_head_tilt_joint",
            "head_2_joint",
        ).value
        self._head_pan_sign = float(
            self.declare_parameter("robot_head_pan_sign", -1.0).value
        )
        self._head_tilt_sign = float(
            self.declare_parameter("robot_head_tilt_sign", -1.0).value
        )
        self._max_state_age_sec = float(
            self.declare_parameter("robot_state_max_age_sec", 1.0).value
        )
        joint_state_topic = self.declare_parameter(
            "robot_joint_state_topic",
            "/joint_states",
        ).value

        self._joint_positions: dict[str, float] = {}
        self._last_joint_state_sec = 0.0
        self._joint_lock = threading.Lock()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            JointState,
            joint_state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Robot input snapshots use "
            f"'{self._reference_frame}' -> '{self._hand_frame}' and "
            f"'{self._reference_frame}' -> '{self._head_frame}'"
        )

    def _on_joint_state(self, message: JointState) -> None:
        with self._joint_lock:
            for name, position in zip(message.name, message.position):
                self._joint_positions[name] = float(position)
            self._last_joint_state_sec = self.get_clock().now().nanoseconds / 1e9

    def get_snapshot(self) -> dict[str, Any]:
        errors: list[str] = []
        now_sec = self.get_clock().now().nanoseconds / 1e9
        with self._joint_lock:
            pan = self._joint_positions.get(self._head_pan_joint)
            tilt = self._joint_positions.get(self._head_tilt_joint)
            joint_state_age_sec = now_sec - self._last_joint_state_sec

        if pan is None or tilt is None:
            errors.append(
                "Head joint state is unavailable for "
                f"'{self._head_pan_joint}' and '{self._head_tilt_joint}'"
            )
        elif joint_state_age_sec > self._max_state_age_sec:
            errors.append(
                f"Head joint state is stale ({joint_state_age_sec:.2f}s old)"
            )

        hand_transform = self._lookup_transform(self._hand_frame, errors)
        head_transform = self._lookup_transform(self._head_frame, errors)

        snapshot: dict[str, Any] = {
            "status": "ok" if not errors else "unavailable",
            "ready": not errors,
            "timestamp": now_sec,
            "referenceFrame": self._reference_frame,
            "errors": errors,
        }

        if pan is not None and tilt is not None and head_transform is not None:
            head_pose = _transform_pose(head_transform)
            snapshot["head"] = {
                "frame": self._reference_frame,
                "sourceFrame": self._head_frame,
                "position": head_pose["position"],
                "orientation": _head_command_quaternion(
                    pan,
                    tilt,
                    self._head_pan_sign,
                    self._head_tilt_sign,
                ),
                "robotOrientation": head_pose["orientation"],
                "pan": pan,
                "tilt": tilt,
                "panSign": self._head_pan_sign,
                "tiltSign": self._head_tilt_sign,
            }

        if hand_transform is not None:
            hand_pose = _transform_pose(hand_transform)
            snapshot["wrist"] = {
                "frame": self._reference_frame,
                "sourceFrame": self._hand_frame,
                **hand_pose,
            }

        return snapshot

    def _lookup_transform(
        self,
        source_frame: str,
        errors: list[str],
    ) -> Any | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._reference_frame,
                source_frame,
                Time(),
            )
            stamp = transform.header.stamp
            stamp_sec = float(stamp.sec) + (float(stamp.nanosec) / 1e9)
            age_sec = (self.get_clock().now().nanoseconds / 1e9) - stamp_sec
            if stamp_sec > 0.0 and age_sec > self._max_state_age_sec:
                errors.append(
                    f"Transform '{self._reference_frame}' -> '{source_frame}' "
                    f"is stale ({age_sec:.2f}s old)"
                )
                return None
            return transform
        except TransformException as error:
            errors.append(
                f"Transform '{self._reference_frame}' -> '{source_frame}' "
                f"is unavailable: {error}"
            )
            return None
