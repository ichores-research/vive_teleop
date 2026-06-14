from typing import Callable

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


class TeleopDataReceiver:
    """Own the ROS subscriptions that feed the teleoperation controllers."""

    def __init__(
        self,
        node: Node,
        *,
        head_input_topic: str,
        hand_target_topic: str,
        hand_target_active_topic: str,
        gripper_input_topic: str,
        joint_state_topic: str,
        on_head_pose: Callable[[PoseStamped], None],
        on_hand_target: Callable[[PoseStamped], None],
        on_hand_target_active: Callable[[Bool], None],
        on_gripper_opening: Callable[[Float64], None],
        on_joint_state: Callable[[JointState], None],
    ) -> None:
        self._subscriptions = [
            node.create_subscription(
                PoseStamped,
                head_input_topic,
                on_head_pose,
                10,
            ),
            node.create_subscription(
                PoseStamped,
                hand_target_topic,
                on_hand_target,
                1,
            ),
            node.create_subscription(
                Bool,
                hand_target_active_topic,
                on_hand_target_active,
                10,
            ),
            node.create_subscription(
                Float64,
                gripper_input_topic,
                on_gripper_opening,
                10,
            ),
            node.create_subscription(
                JointState,
                joint_state_topic,
                on_joint_state,
                qos_profile_sensor_data,
            ),
        ]
