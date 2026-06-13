import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(package_name: str, relative_path: str) -> dict:
    package_share = get_package_share_directory(package_name)
    with open(
        os.path.join(package_share, relative_path),
        "r",
        encoding="utf-8",
    ) as yaml_file:
        return yaml.safe_load(yaml_file)


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
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            output="screen",
            parameters=[
                servo_params,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                robot_model_params,
            ],
        ),
        Node(
            package="vive_moveit_server",
            executable="servo_pose_bridge",
            name="servo_pose_bridge",
            output="screen",
            parameters=[
                os.path.join(
                    get_package_share_directory("vive_moveit_server"),
                    "config",
                    "servo_pose_bridge.yaml",
                )
            ],
        ),
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
