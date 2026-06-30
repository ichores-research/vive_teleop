from setuptools import setup

package_name = "image_listener"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mateusz Wątły",
    maintainer_email="mateuszwatly@gmail.com",
    description="ROS 2 WebRTC video, input, and robot-state gateway.",
    license="Apache-2.0",
    url="https://github.com/ichores-research/vive_teleop",
    entry_points={
        "console_scripts": [
            "image_subscriber = image_listener.image_subscriber:main",
            "teleop_webrtc = image_listener.teleop_webrtc:main",
        ],
    },
)
