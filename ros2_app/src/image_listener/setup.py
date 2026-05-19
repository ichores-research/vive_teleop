from setuptools import setup

package_name = "image_listener"

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
    description="Image subscriber node",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "image_subscriber = image_listener.image_subscriber:main",
        ],
    },
)
