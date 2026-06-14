from glob import glob

from setuptools import setup

package_name = "vive_moveit_server"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="mateuszwatly@gmail.com",
    description="ROS 2 MoveIt teleoperation server for Vive WebRTC input.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vive_moveit_server = vive_moveit_server.vive_moveit_server:main",
            "servo_pose_bridge = vive_moveit_server.servo_pose_bridge:main",
        ],
    },
)
