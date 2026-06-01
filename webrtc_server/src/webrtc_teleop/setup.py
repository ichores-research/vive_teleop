from setuptools import setup

package_name = "webrtc_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="mateuszwatly@gmail.com",
    description="WebRTC video and teleop input bridge",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "image_subscriber = webrtc_teleop.image_subscriber:main",
            "teleop_webrtc = webrtc_teleop.teleop_webrtc:main",
        ],
    },
)
