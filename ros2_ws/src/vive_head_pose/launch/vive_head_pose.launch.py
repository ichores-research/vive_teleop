"""Launch Vive HMD pose publisher (ROS 2 Foxy)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/vive/head_pose',
            description='geometry_msgs/PoseStamped output topic',
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='vive_world',
            description='Parent frame for pose and TF',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='vive_hmd',
            description='Child frame (headset)',
        ),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='90.0',
            description='Poll/publish rate',
        ),
        Node(
            package='vive_head_pose',
            executable='vive_head_pose_node',
            name='vive_head_pose',
            output='screen',
            parameters=[{
                'pose_topic': LaunchConfiguration('pose_topic'),
                'world_frame': LaunchConfiguration('world_frame'),
                'frame_id': LaunchConfiguration('frame_id'),
                'rate_hz': LaunchConfiguration('rate_hz'),
                'publish_tf': True,
                'tracking_universe': 'standing',
            }],
        ),
    ])
