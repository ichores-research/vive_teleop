import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TIAGO_VARIANT_ARGUMENTS = (
    ("moveit_use_sim_time", "use_sim_time"),
    ("moveit_arm_type", "arm_type"),
    ("moveit_base_type", "base_type"),
    ("moveit_end_effector", "end_effector"),
    ("moveit_ft_sensor", "ft_sensor"),
    ("moveit_wrist_model", "wrist_model"),
    ("moveit_camera_model", "camera_model"),
    ("moveit_laser_model", "laser_model"),
)

ROBOT_DESCRIPTION_ARGUMENTS = (
    *TIAGO_VARIANT_ARGUMENTS,
    ("moveit_has_screen", "has_screen"),
    ("moveit_is_public_sim", "is_public_sim"),
    ("moveit_namespace", "namespace"),
    ("moveit_gazebo_version", "gazebo_version"),
)


def is_truthy(value):
    return value.strip().lower() not in ("", "0", "false", "no", "off")


def optional_launch_argument(context, local_name, included_name):
    value = LaunchConfiguration(local_name).perform(context)
    if not value:
        return None
    return (included_name, value)


def launch_path_or_message(package, launch_file, missing_message):
    if os.path.isabs(launch_file):
        launch_path = launch_file
    else:
        if not package:
            return None, [
                LogInfo(
                    msg=missing_message("No launch package configured")
                )
            ]

        try:
            package_share = get_package_share_directory(package)
        except PackageNotFoundError:
            return None, [
                LogInfo(
                    msg=missing_message(
                        f"Launch package '{package}' was not found"
                    )
                )
            ]

        launch_path = os.path.join(
            package_share,
            "launch",
            launch_file,
        )

    if not os.path.exists(launch_path):
        return None, [
            LogInfo(
                msg=missing_message(
                    f"Launch file '{launch_path}' was not found"
                )
            )
        ]

    return launch_path, []


def include_optional_robot_description_launch(context, *args, **kwargs):
    enabled = LaunchConfiguration("robot_description_launch_enabled").perform(context)
    if not is_truthy(enabled):
        return [
            LogInfo(
                msg="Robot description launch include disabled; MoveIt must receive /robot_description from another ROS2 publisher."
            )
        ]

    package = LaunchConfiguration("robot_description_launch_package").perform(context)
    launch_file = LaunchConfiguration("robot_description_launch_file").perform(context)
    if not launch_file:
        return [
            LogInfo(
                msg="No robot description launch file configured; MoveIt must receive /robot_description from another ROS2 publisher."
            )
        ]

    launch_path, messages = launch_path_or_message(
        package,
        launch_file,
        lambda reason: (
            reason
            + "; MoveIt must receive /robot_description from another ROS2 publisher."
        ),
    )
    if launch_path is None:
        return messages

    included_arguments = []
    for local_name, included_name in ROBOT_DESCRIPTION_ARGUMENTS:
        launch_argument = optional_launch_argument(context, local_name, included_name)
        if launch_argument is not None:
            included_arguments.append(launch_argument)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=included_arguments,
        )
    ]


def include_optional_moveit_launch(context, *args, **kwargs):
    enabled = LaunchConfiguration("moveit_launch_enabled").perform(context)
    if not is_truthy(enabled):
        return [
            LogInfo(
                msg="MoveIt launch include disabled; vive_moveit_server will wait for an external MoveGroup action server."
            )
        ]

    package = LaunchConfiguration("moveit_launch_package").perform(context)
    launch_file = LaunchConfiguration("moveit_launch_file").perform(context)
    if not launch_file:
        return [
            LogInfo(
                msg="No MoveIt launch file configured; vive_moveit_server will wait for an external MoveGroup action server."
            )
        ]

    launch_path, messages = launch_path_or_message(
        package,
        launch_file,
        lambda reason: (
            reason
            + "; vive_moveit_server will wait for an external MoveGroup action server."
        ),
    )
    if launch_path is None:
        return messages

    included_arguments = []
    for local_name, included_name in (
        ("moveit_allow_trajectory_execution", "allow_trajectory_execution"),
        ("moveit_publish_monitored_planning_scene", "publish_monitored_planning_scene"),
        ("moveit_arm", "arm"),
        *TIAGO_VARIANT_ARGUMENTS,
        ("moveit_use_sensor_manager", "use_sensor_manager"),
    ):
        launch_argument = optional_launch_argument(context, local_name, included_name)
        if launch_argument is not None:
            included_arguments.append(launch_argument)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=included_arguments,
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
                "moveit_launch_enabled",
                default_value="true",
                description="Start the configured ROS2 MoveIt move_group launch before teleop.",
            ),
            DeclareLaunchArgument(
                "moveit_launch_package",
                default_value="tiago_moveit_config",
                description="MoveIt config package to include before teleop.",
            ),
            DeclareLaunchArgument(
                "moveit_launch_file",
                default_value="move_group.launch.py",
                description="MoveIt launch file name or absolute path.",
            ),
            DeclareLaunchArgument(
                "robot_description_launch_enabled",
                default_value="true",
                description="Start the configured robot_state_publisher launch so MoveIt receives /robot_description.",
            ),
            DeclareLaunchArgument(
                "robot_description_launch_package",
                default_value="tiago_description",
                description="Robot description package to include before MoveIt.",
            ),
            DeclareLaunchArgument(
                "robot_description_launch_file",
                default_value="robot_state_publisher.launch.py",
                description="Robot description launch file name or absolute path.",
            ),
            DeclareLaunchArgument(
                "moveit_allow_trajectory_execution",
                default_value="False",
                description="Forwarded to MoveIt move_group; false keeps execution in vive_moveit_server controller-topic mode.",
            ),
            DeclareLaunchArgument(
                "moveit_publish_monitored_planning_scene",
                default_value="True",
                description="Forwarded to MoveIt move_group.",
            ),
            DeclareLaunchArgument(
                "moveit_use_sim_time",
                default_value="False",
                description="Forwarded to MoveIt move_group.",
            ),
            DeclareLaunchArgument(
                "moveit_arm",
                default_value="",
                description="Optional arm forwarded to the included TIAGo MoveIt launch.",
            ),
            DeclareLaunchArgument(
                "moveit_arm_type",
                default_value="",
                description="Optional arm_type forwarded to the included TIAGo MoveIt launch.",
            ),
            DeclareLaunchArgument(
                "moveit_base_type",
                default_value="",
                description="Optional base_type forwarded to the included TIAGo MoveIt launch.",
            ),
            DeclareLaunchArgument(
                "moveit_end_effector",
                default_value="",
                description="Optional end_effector forwarded to the included TIAGo MoveIt launch.",
            ),
            DeclareLaunchArgument(
                "moveit_ft_sensor",
                default_value="",
                description="Optional ft_sensor forwarded to the included TIAGo MoveIt launch.",
            ),
            DeclareLaunchArgument(
                "moveit_wrist_model",
                default_value="",
                description="Optional wrist_model forwarded to the included TIAGo launches.",
            ),
            DeclareLaunchArgument(
                "moveit_camera_model",
                default_value="",
                description="Optional camera_model forwarded to the included TIAGo launches.",
            ),
            DeclareLaunchArgument(
                "moveit_laser_model",
                default_value="",
                description="Optional laser_model forwarded to the included TIAGo launches.",
            ),
            DeclareLaunchArgument(
                "moveit_has_screen",
                default_value="",
                description="Optional has_screen forwarded to the included TIAGo robot description launch.",
            ),
            DeclareLaunchArgument(
                "moveit_is_public_sim",
                default_value="",
                description="Optional is_public_sim forwarded to the included TIAGo robot description launch.",
            ),
            DeclareLaunchArgument(
                "moveit_namespace",
                default_value="",
                description="Optional namespace forwarded to the included TIAGo robot description launch.",
            ),
            DeclareLaunchArgument(
                "moveit_gazebo_version",
                default_value="",
                description="Optional gazebo_version forwarded to the included TIAGo robot description launch.",
            ),
            DeclareLaunchArgument(
                "moveit_use_sensor_manager",
                default_value="",
                description="Optional use_sensor_manager forwarded to the included TIAGo MoveIt launch.",
            ),
            OpaqueFunction(function=include_optional_robot_description_launch),
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
