import os
from glob import glob

from setuptools import find_packages, setup


package_name = "velocity_servo_tag"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config", "urdf"),
            glob("config/urdf/*.urdf"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="harry",
    maintainer_email="harry@todo.todo",
    description=(
        "AprilTag detection and camera-to-joint velocity conversion for FR3."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "apriltag_detector = "
                "velocity_servo_tag.vision.apriltag_detector:main"
            ),
            (
                "velocity_mapper_node = "
                "velocity_servo_tag.velocity_mapper_node:main"
            ),
        ],
    },
)
