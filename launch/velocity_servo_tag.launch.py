#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("velocity_servo_tag")
    default_params = os.path.join(
        package_share,
        "config",
        "velocity_servo_tag.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    dry_run = LaunchConfiguration("dry_run")
    start_detector = LaunchConfiguration("start_detector")

    detector_node = Node(
        package="velocity_servo_tag",
        executable="apriltag_detector",
        name="apriltag_detector",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_detector),
        parameters=[params_file],
    )

    velocity_node = Node(
        package="velocity_servo_tag",
        executable="velocity_mapper_node",
        name="velocity_mapper_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            params_file,
            {
                "dry_run": ParameterValue(
                    dry_run,
                    value_type=bool,
                )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS 2 parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="Do not publish joint commands when true.",
            ),
            DeclareLaunchArgument(
                "start_detector",
                default_value="true",
                description="Start the USB AprilTag detector.",
            ),
            detector_node,
            velocity_node,
        ]
    )
