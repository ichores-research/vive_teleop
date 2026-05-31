import math
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _vector_parameter(value: object, fallback: list[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return [float(item) for item in value]
    return list(fallback)


def _string_list_parameter(value: object, fallback: list[str]) -> list[str]:
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value if str(item)]
    return list(fallback)


def _declare_string_list_parameter(
    node: Node,
    name: str,
    fallback: list[str],
) -> list[str]:
    default_value = fallback if fallback else Parameter.Type.STRING_ARRAY
    return _string_list_parameter(
        node.declare_parameter(name, default_value).value,
        fallback,
    )


def _select_joint_values(
    values: object,
    selected_indices: list[int],
    source_joint_count: int,
) -> list[float]:
    if len(values) != source_joint_count:
        return []
    return [float(values[index]) for index in selected_indices]


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


def _position_distance(left: PoseStamped, right: PoseStamped) -> float:
    dx = left.pose.position.x - right.pose.position.x
    dy = left.pose.position.y - right.pose.position.y
    dz = left.pose.position.z - right.pose.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _position_norm(message: PoseStamped) -> float:
    position = message.pose.position
    return math.sqrt(
        (position.x * position.x)
        + (position.y * position.y)
        + (position.z * position.z)
    )


def _orientation_distance(left: PoseStamped, right: PoseStamped) -> float:
    lq = left.pose.orientation
    rq = right.pose.orientation
    dot = abs((lq.x * rq.x) + (lq.y * rq.y) + (lq.z * rq.z) + (lq.w * rq.w))
    dot = max(0.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


class ViveMoveItServer(Node):
    def __init__(self) -> None:
        super().__init__("vive_moveit_server")

        self.arm_group = self.declare_parameter("arm_group", "arm_torso").value
        self.end_effector_link = self.declare_parameter(
            "end_effector_link",
            "arm_tool_link",
        ).value

        head_input_topic = self.declare_parameter(
            "head_input_topic",
            "/vive/head_pose",
        ).value
        head_output_topic = self.declare_parameter(
            "head_output_topic",
            "/vive/robot_head_pose",
        ).value
        head_extra_output_topics = _declare_string_list_parameter(
            self,
            "head_extra_output_topics",
            [],
        )
        hand_target_topic = self.declare_parameter(
            "hand_target_topic",
            "/vive/hand_target_pose",
        ).value
        arm_command_topic = self.declare_parameter(
            "arm_command_topic",
            "/arm_controller/command",
        ).value
        torso_command_topic = self.declare_parameter(
            "torso_command_topic",
            "/torso_controller/command",
        ).value
        self.move_group_action_name = self.declare_parameter(
            "move_group_action_name",
            "/move_action",
        ).value

        self.execution_mode = self.declare_parameter("execution_mode", "moveit").value
        self.async_execution = bool(
            self.declare_parameter("async_execution", True).value
        )
        self.pose_reference_frame = self.declare_parameter(
            "pose_reference_frame",
            "base_footprint",
        ).value
        self.planning_time_sec = float(
            self.declare_parameter("planning_time_sec", 0.35).value
        )
        self.max_velocity_scaling_factor = float(
            self.declare_parameter("max_velocity_scaling_factor", 0.25).value
        )
        self.max_acceleration_scaling_factor = float(
            self.declare_parameter("max_acceleration_scaling_factor", 0.25).value
        )
        self.min_plan_interval_sec = float(
            self.declare_parameter("min_plan_interval_sec", 0.25).value
        )
        self.position_deadband_m = float(
            self.declare_parameter("position_deadband_m", 0.01).value
        )
        self.orientation_deadband_rad = float(
            self.declare_parameter("orientation_deadband_rad", 0.035).value
        )
        self.goal_position_tolerance_m = float(
            self.declare_parameter("goal_position_tolerance_m", 0.01).value
        )
        self.goal_orientation_tolerance_rad = float(
            self.declare_parameter("goal_orientation_tolerance_rad", 0.035).value
        )
        self.num_planning_attempts = int(
            self.declare_parameter("num_planning_attempts", 1).value
        )
        self.wait_for_move_group_timeout_sec = float(
            self.declare_parameter("wait_for_move_group_timeout_sec", 0.0).value
        )
        self.max_hand_target_distance_m = float(
            self.declare_parameter("max_hand_target_distance_m", 1.5).value
        )
        self.min_hand_target_z_m = float(
            self.declare_parameter("min_hand_target_z_m", 0.2).value
        )
        self.max_hand_target_z_m = float(
            self.declare_parameter("max_hand_target_z_m", 1.6).value
        )

        self.hand_position_scale = _vector_parameter(
            self.declare_parameter(
                "hand_position_scale",
                [1.0, 1.0, 1.0],
            ).value,
            [1.0, 1.0, 1.0],
        )
        self.hand_position_offset = _vector_parameter(
            self.declare_parameter(
                "hand_position_offset",
                [0.0, 0.0, 0.0],
            ).value,
            [0.0, 0.0, 0.0],
        )
        self.arm_joint_names = _string_list_parameter(
            self.declare_parameter(
                "arm_joint_names",
                [
                    "arm_1_joint",
                    "arm_2_joint",
                    "arm_3_joint",
                    "arm_4_joint",
                    "arm_5_joint",
                    "arm_6_joint",
                    "arm_7_joint",
                ],
            ).value,
            [
                "arm_1_joint",
                "arm_2_joint",
                "arm_3_joint",
                "arm_4_joint",
                "arm_5_joint",
                "arm_6_joint",
                "arm_7_joint",
            ],
        )
        self.torso_joint_names = _string_list_parameter(
            self.declare_parameter(
                "torso_joint_names",
                ["torso_lift_joint"],
            ).value,
            ["torso_lift_joint"],
        )

        self.pending_hand_target: Optional[PoseStamped] = None
        self.last_commanded_target: Optional[PoseStamped] = None
        self.last_plan_started_sec = 0.0
        self.last_log_times: Dict[str, float] = {}
        self.goal_in_flight = False
        self.received_head_pose = False
        self.received_hand_target = False

        self.head_output_topics = []
        for topic in [head_output_topic, *head_extra_output_topics]:
            if topic and topic not in self.head_output_topics:
                self.head_output_topics.append(topic)
        self.head_publishers = [
            self.create_publisher(PoseStamped, topic, 10)
            for topic in self.head_output_topics
        ]
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            arm_command_topic,
            10,
        )
        self.torso_trajectory_publisher = None
        if torso_command_topic:
            self.torso_trajectory_publisher = self.create_publisher(
                JointTrajectory,
                torso_command_topic,
                10,
            )
        self.move_group_action = ActionClient(
            self,
            MoveGroup,
            self.move_group_action_name,
        )

        self.create_subscription(
            PoseStamped,
            head_input_topic,
            self._on_head_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            hand_target_topic,
            self._on_hand_target,
            10,
        )
        self.create_timer(0.02, self._maybe_send_latest_target)

        self.get_logger().info(
            f"Listening for head poses on '{head_input_topic}' and hand targets "
            f"on '{hand_target_topic}'"
        )
        self.get_logger().info(
            f"Head poses are forwarded unchanged to '{head_output_topic}'"
        )
        self.get_logger().info(
            f"MoveIt action '{self.move_group_action_name}', group "
            f"'{self.arm_group}', end effector '{self.end_effector_link}', "
            f"mode '{self.execution_mode}'"
        )
        if self.execution_mode == "trajectory_topic":
            self.get_logger().info(
                f"Planned arm trajectories publish to '{arm_command_topic}'"
                + (
                    f" and torso trajectories publish to '{torso_command_topic}'"
                    if torso_command_topic
                    else ""
                )
            )

    def _on_head_pose(self, message: PoseStamped) -> None:
        if not self.received_head_pose:
            self.get_logger().info(
                "Received first head pose input; forwarding to "
                + ", ".join(self.head_output_topics)
            )
            self.received_head_pose = True
        for publisher in self.head_publishers:
            publisher.publish(message)

    def _on_hand_target(self, message: PoseStamped) -> None:
        if not self.received_hand_target:
            self.get_logger().info(
                "Received first hand target input "
                f"frame='{message.header.frame_id}' "
                f"xyz=({message.pose.position.x:.3f}, "
                f"{message.pose.position.y:.3f}, "
                f"{message.pose.position.z:.3f})"
            )
            self.received_hand_target = True
        self.pending_hand_target = message

    def _maybe_send_latest_target(self) -> None:
        if self.goal_in_flight or self.pending_hand_target is None:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_plan_started_sec < self.min_plan_interval_sec:
            return

        if not self.move_group_action.wait_for_server(
            timeout_sec=self.wait_for_move_group_timeout_sec
        ):
            self._warn_throttled(
                "move_group_wait",
                f"Waiting for MoveIt action server '{self.move_group_action_name}'",
                5.0,
            )
            return

        target = self._apply_hand_target_adjustments(self.pending_hand_target)
        if self.pose_reference_frame:
            target.header.frame_id = self.pose_reference_frame

        if not _normalize_quaternion(target.pose.orientation):
            self._warn_throttled(
                "invalid_quaternion",
                "Ignoring hand target with invalid orientation quaternion",
                2.0,
            )
            self.pending_hand_target = None
            return

        if not self._inside_hand_workspace(target):
            self.pending_hand_target = None
            return

        if self._inside_deadband(target):
            self.pending_hand_target = None
            return

        goal = self._build_move_group_goal(target)
        self.goal_in_flight = True
        self.last_plan_started_sec = now_sec
        self.pending_hand_target = None
        self.get_logger().info(
            f"Sending hand target to MoveIt frame='{target.header.frame_id}' "
            f"xyz=({target.pose.position.x:.3f}, "
            f"{target.pose.position.y:.3f}, "
            f"{target.pose.position.z:.3f})"
        )
        send_future = self.move_group_action.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_goal_response(future, target)
        )

    def _apply_hand_target_adjustments(self, message: PoseStamped) -> PoseStamped:
        target = PoseStamped()
        target.header = message.header
        target.pose = message.pose
        target.pose.position.x = (
            target.pose.position.x * self.hand_position_scale[0]
        ) + self.hand_position_offset[0]
        target.pose.position.y = (
            target.pose.position.y * self.hand_position_scale[1]
        ) + self.hand_position_offset[1]
        target.pose.position.z = (
            target.pose.position.z * self.hand_position_scale[2]
        ) + self.hand_position_offset[2]
        return target

    def _inside_deadband(self, target: PoseStamped) -> bool:
        if self.last_commanded_target is None:
            return False

        return (
            _position_distance(target, self.last_commanded_target)
            < self.position_deadband_m
            and _orientation_distance(target, self.last_commanded_target)
            < self.orientation_deadband_rad
        )

    def _inside_hand_workspace(self, target: PoseStamped) -> bool:
        position = target.pose.position
        distance_m = _position_norm(target)
        if (
            self.max_hand_target_distance_m > 0.0
            and distance_m > self.max_hand_target_distance_m
        ):
            self._warn_throttled(
                "target_distance_limit",
                f"Ignoring hand target {distance_m:.3f}m from base; "
                f"max_hand_target_distance_m={self.max_hand_target_distance_m:.3f}. "
                "Check wrist units/calibration.",
                2.0,
            )
            return False

        if (
            self.min_hand_target_z_m < self.max_hand_target_z_m
            and (
                position.z < self.min_hand_target_z_m
                or position.z > self.max_hand_target_z_m
            )
        ):
            self._warn_throttled(
                "target_z_limit",
                f"Ignoring hand target z={position.z:.3f}; expected "
                f"{self.min_hand_target_z_m:.3f} <= z <= "
                f"{self.max_hand_target_z_m:.3f}. Check wrist calibration.",
                2.0,
            )
            return False

        return True

    def _build_move_group_goal(self, target: PoseStamped) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request = self._build_motion_plan_request(target)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = self.execution_mode in (
            "plan_only",
            "trajectory_topic",
        )
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return goal

    def _build_motion_plan_request(self, target: PoseStamped) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self.arm_group
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time_sec
        request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = (
            self.max_acceleration_scaling_factor
        )
        request.start_state = RobotState()
        request.start_state.is_diff = True
        request.goal_constraints = [self._build_goal_constraints(target)]
        return request

    def _build_goal_constraints(self, target: PoseStamped) -> Constraints:
        constraints = Constraints()
        constraints.name = "vive_hand_target"

        position_constraint = PositionConstraint()
        position_constraint.header = target.header
        position_constraint.link_name = self.end_effector_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.goal_position_tolerance_m]
        sphere_pose = Pose()
        sphere_pose.position = target.pose.position
        sphere_pose.orientation.w = 1.0
        region = BoundingVolume()
        region.primitives = [sphere]
        region.primitive_poses = [sphere_pose]
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = (
            self.goal_orientation_tolerance_rad
        )
        orientation_constraint.absolute_y_axis_tolerance = (
            self.goal_orientation_tolerance_rad
        )
        orientation_constraint.absolute_z_axis_tolerance = (
            self.goal_orientation_tolerance_rad
        )
        orientation_constraint.weight = 1.0

        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        return constraints

    def _on_goal_response(
        self,
        future: Future,
        target: PoseStamped,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().warn(f"MoveIt goal request failed: {error}")
            self.goal_in_flight = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("MoveIt rejected the hand target goal")
            self.goal_in_flight = False
            return

        self.get_logger().info("MoveIt accepted hand target goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_goal_result(result, target)
        )

    def _on_goal_result(self, future: Future, target: PoseStamped) -> None:
        try:
            action_result = future.result().result
        except Exception as error:
            self.get_logger().warn(f"MoveIt action failed: {error}")
            self.goal_in_flight = False
            return

        error_code = action_result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self._warn_throttled(
                "moveit_error",
                f"MoveIt could not satisfy hand target, error_code={error_code}",
                2.0,
            )
            self.goal_in_flight = False
            return

        if self.execution_mode == "trajectory_topic":
            self._publish_planned_trajectory(
                action_result.planned_trajectory.joint_trajectory
            )
        elif self.execution_mode == "plan_only":
            self._info_throttled(
                "plan_only_success",
                "MoveIt plan succeeded; execution_mode=plan_only",
                2.0,
            )

        self.last_commanded_target = target
        self.goal_in_flight = False

    def _publish_planned_trajectory(self, trajectory: JointTrajectory) -> None:
        if not trajectory.joint_names:
            self._warn_throttled(
                "empty_trajectory",
                "MoveIt returned an empty planned joint trajectory",
                2.0,
            )
            return

        published_topics = []
        arm_trajectory = self._filter_joint_trajectory(
            trajectory,
            self.arm_joint_names,
        )
        if arm_trajectory is not None:
            self.trajectory_publisher.publish(arm_trajectory)
            published_topics.append("arm")

        if self.torso_trajectory_publisher is not None:
            torso_trajectory = self._filter_joint_trajectory(
                trajectory,
                self.torso_joint_names,
            )
            if torso_trajectory is not None:
                self.torso_trajectory_publisher.publish(torso_trajectory)
                published_topics.append("torso")

        if published_topics:
            self._info_throttled(
                "trajectory_topic_success",
                f"Published planned trajectory segments: {', '.join(published_topics)} "
                f"joints={list(trajectory.joint_names)} points={len(trajectory.points)}",
                2.0,
            )
            return

        self._warn_throttled(
            "trajectory_no_matching_joints",
            "MoveIt planned joints "
            f"{list(trajectory.joint_names)} but none match configured arm/torso joint names",
            2.0,
        )

    def _filter_joint_trajectory(
        self,
        trajectory: JointTrajectory,
        selected_joint_names: list[str],
    ) -> Optional[JointTrajectory]:
        selected_indices = [
            index
            for index, joint_name in enumerate(trajectory.joint_names)
            if joint_name in selected_joint_names
        ]
        if not selected_indices:
            return None

        filtered = JointTrajectory()
        filtered.header = trajectory.header
        filtered.joint_names = [
            trajectory.joint_names[index] for index in selected_indices
        ]
        source_joint_count = len(trajectory.joint_names)

        for point in trajectory.points:
            filtered_point = JointTrajectoryPoint()
            filtered_point.positions = _select_joint_values(
                point.positions,
                selected_indices,
                source_joint_count,
            )
            filtered_point.velocities = _select_joint_values(
                point.velocities,
                selected_indices,
                source_joint_count,
            )
            filtered_point.accelerations = _select_joint_values(
                point.accelerations,
                selected_indices,
                source_joint_count,
            )
            filtered_point.effort = _select_joint_values(
                point.effort,
                selected_indices,
                source_joint_count,
            )
            filtered_point.time_from_start = point.time_from_start
            filtered.points.append(filtered_point)

        return filtered

    def _warn_throttled(self, key: str, message: str, period_sec: float) -> None:
        if self._should_log(key, period_sec):
            self.get_logger().warn(message)

    def _info_throttled(self, key: str, message: str, period_sec: float) -> None:
        if self._should_log(key, period_sec):
            self.get_logger().info(message)

    def _should_log(self, key: str, period_sec: float) -> bool:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        last_sec = self.last_log_times.get(key)
        if last_sec is not None and now_sec - last_sec < period_sec:
            return False

        self.last_log_times[key] = now_sec
        return True


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ViveMoveItServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
