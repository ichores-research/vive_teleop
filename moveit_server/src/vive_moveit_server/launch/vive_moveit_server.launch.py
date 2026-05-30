import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_optional_moveit_launch(context, *args, **kwargs):
    package = LaunchConfiguration("moveit_launch_package").perform(context)
    launch_file = LaunchConfiguration("moveit_launch_file").perform(context)
    if not package or not launch_file:
        return []

    if os.path.isabs(launch_file):
        launch_path = launch_file
    else:
        launch_path = os.path.join(
            get_package_share_directory(package),
            "launch",
            launch_file,
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
        )
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("vive_moveit_server"),
            "config",
            "tiago_single_params.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameter file for vive_moveit_server.",
            ),
            DeclareLaunchArgument(
                "moveit_launch_package",
                default_value="",
                description="Optional MoveIt config package to include before teleop.",
            ),
            DeclareLaunchArgument(
                "moveit_launch_file",
                default_value="",
                description="Optional MoveIt launch file name or absolute path.",
            ),
            OpaqueFunction(function=include_optional_moveit_launch),
            Node(
                package="vive_moveit_server",
                executable="vive_moveit_server",
                name="vive_moveit_server",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
