import os
import xml.etree.ElementTree as ET

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


ARM_GROUP_NAME = "arm"
EXPECTED_ARM_JOINTS = tuple(f"arm_{index}_joint" for index in range(1, 8))


def _load_yaml(package_name: str, relative_path: str) -> dict:
    package_share = get_package_share_directory(package_name)
    with open(
        os.path.join(package_share, relative_path),
        "r",
        encoding="utf-8",
    ) as yaml_file:
        return yaml.safe_load(yaml_file)


def _validate_arm_group(srdf_path: str, mappings: dict[str, str]) -> None:
    srdf_xml = xacro.process_file(srdf_path, mappings=mappings).toxml()
    root = ET.fromstring(srdf_xml)
    arm_group = next(
        (
            group
            for group in root.findall("group")
            if group.get("name") == ARM_GROUP_NAME
        ),
        None,
    )
    if arm_group is None:
        raise RuntimeError(
            f"MoveIt SRDF does not define the '{ARM_GROUP_NAME}' group"
        )

    group_joints = {
        joint.get("name")
        for joint in arm_group.findall("joint")
        if joint.get("name")
    }
    missing_joints = [
        joint_name
        for joint_name in EXPECTED_ARM_JOINTS
        if joint_name not in group_joints
    ]
    if missing_joints:
        raise RuntimeError(
            f"MoveIt group '{ARM_GROUP_NAME}' is not the full 7-DOF arm; "
            f"missing joints: {', '.join(missing_joints)}"
        )
    if "torso_lift_joint" in group_joints:
        raise RuntimeError(
            f"MoveIt group '{ARM_GROUP_NAME}' unexpectedly includes "
            "'torso_lift_joint'"
        )

    get_logger("servo_runtime").info(
        "Validated MoveIt Servo group 'arm' with arm_1_joint through "
        "arm_7_joint"
    )


def _start_servo_runtime(context, *args, **kwargs):
    moveit_package = get_package_share_directory("tiago_moveit_config")
    srdf_path = os.path.join(
        moveit_package,
        "config",
        "srdf",
        "tiago.srdf.xacro",
    )
    srdf_mappings = {
        "arm_type": LaunchConfiguration("arm_type").perform(context),
        "end_effector": LaunchConfiguration("end_effector").perform(context),
        "ft_sensor": LaunchConfiguration("ft_sensor").perform(context),
        "base_type": LaunchConfiguration("base_type").perform(context),
    }
    _validate_arm_group(srdf_path, srdf_mappings)
    moveit_config = (
        MoveItConfigsBuilder("tiago")
        .robot_description_semantic(
            file_path=srdf_path,
            mappings=srdf_mappings,
        )
        .robot_description_kinematics(
            file_path="config/kinematics_kdl.yaml"
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    runtime_yaml = _load_yaml(
        "vive_moveit_server",
        "config/tiago_servo.yaml",
    )
    servo_params = {"moveit_servo": runtime_yaml["moveit_servo"]}
    robot_model_params = {
        "robot_description_timeout": 60.0,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    return [
        ComposableNodeContainer(
            name="servo_node_container",
            namespace="/",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="moveit_servo",
                    plugin="moveit_servo::ServoNode",
                    name="servo_node",
                    parameters=[
                        servo_params,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        moveit_config.joint_limits,
                        robot_model_params,
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("base_type", default_value="pmb2"),
            DeclareLaunchArgument("arm_type", default_value="tiago-arm"),
            DeclareLaunchArgument(
                "end_effector",
                default_value="pal-gripper",
            ),
            DeclareLaunchArgument("ft_sensor", default_value="schunk-ft"),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            OpaqueFunction(function=_start_servo_runtime),
        ]
    )
