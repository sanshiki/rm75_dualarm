# ================================================================
# Image white-balance correction
# ================================================================
# Applies offline RGB gains to raw camera image topics and republishes
# corrected sensor_msgs/Image streams for downstream consumers.
#
# Usage:
#   ros2 launch rm_dualarm white_balance.launch.py
#   ros2 launch rm_dualarm white_balance.launch.py camera_output_topic:=/la/main/image
# ================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _white_balance_node(name, camera_key, input_arg, output_arg):
    return Node(
        package="rm_dualarm",
        executable="image_white_balance_node.py",
        name=name,
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration(input_arg),
                "output_topic": LaunchConfiguration(output_arg),
                "calibration_file": LaunchConfiguration("white_balance_config"),
                "camera_key": camera_key,
                "queue_size": LaunchConfiguration("queue_size"),
                "input_reliability": LaunchConfiguration("input_reliability"),
                "output_reliability": LaunchConfiguration("output_reliability"),
            }
        ],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("rm_dualarm")
    default_config = os.path.join(pkg_share, "config", "white_balance.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("white_balance_config", default_value=default_config),
            DeclareLaunchArgument("queue_size", default_value="5"),
            DeclareLaunchArgument("input_reliability", default_value="best_effort"),
            DeclareLaunchArgument("output_reliability", default_value="reliable"),
            DeclareLaunchArgument("camera_input_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_output_topic", default_value="/camera/image_wb"),
            DeclareLaunchArgument(
                "wrist_camera_input_topic",
                default_value="/wrist_camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "wrist_camera_output_topic",
                default_value="/wrist_camera/color/image_wb",
            ),
            _white_balance_node(
                "camera_white_balance",
                "camera",
                "camera_input_topic",
                "camera_output_topic",
            ),
            _white_balance_node(
                "wrist_camera_white_balance",
                "wrist_camera",
                "wrist_camera_input_topic",
                "wrist_camera_output_topic",
            ),
        ]
    )
