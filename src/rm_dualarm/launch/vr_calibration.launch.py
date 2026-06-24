from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vr_base_frame", default_value="vr_base"),
        DeclareLaunchArgument("left_vr_hand_frame", default_value="hand_left"),
        DeclareLaunchArgument("right_vr_hand_frame", default_value="hand_right"),
        DeclareLaunchArgument("joy_topic", default_value="/quest/joystick"),
        DeclareLaunchArgument("output_file", default_value="/tmp/vr_calibration.yaml"),
        Node(
            package="rm_dualarm",
            executable="vr_calibration_capture.py",
            name="vr_calibration_capture",
            output="screen",
            parameters=[{
                "vr_base_frame": LaunchConfiguration("vr_base_frame"),
                "left_vr_hand_frame": LaunchConfiguration("left_vr_hand_frame"),
                "right_vr_hand_frame": LaunchConfiguration("right_vr_hand_frame"),
                "joy_topic": LaunchConfiguration("joy_topic"),
                "output_file": LaunchConfiguration("output_file"),
            }],
        ),
    ])
