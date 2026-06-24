from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vr_base_frame", default_value="vr_base"),
        DeclareLaunchArgument("vr_origin_frame", default_value="vr_origin"),
        DeclareLaunchArgument("headset_frame", default_value="headset"),
        DeclareLaunchArgument("left_vr_hand_frame", default_value="hand_left"),
        DeclareLaunchArgument("right_vr_hand_frame", default_value="hand_right"),
        DeclareLaunchArgument("left_base_frame", default_value="left_base_link"),
        DeclareLaunchArgument("right_base_frame", default_value="right_base_link"),
        DeclareLaunchArgument("left_ee_frame", default_value="left_Link7"),
        DeclareLaunchArgument("right_ee_frame", default_value="right_Link7"),
        DeclareLaunchArgument("joy_topic", default_value="/quest/joystick"),
        DeclareLaunchArgument("output_file", default_value="/tmp/vr_calibration.yaml"),
        DeclareLaunchArgument("sample_duration", default_value="2.0"),
        DeclareLaunchArgument("sample_rate", default_value="50.0"),
        DeclareLaunchArgument("mirror_convergence", default_value="false"),
        Node(
            package="rm_dualarm",
            executable="vr_calibration_capture.py",
            name="vr_calibration_capture",
            output="screen",
            parameters=[{
                "vr_base_frame": LaunchConfiguration("vr_base_frame"),
                "vr_origin_frame": LaunchConfiguration("vr_origin_frame"),
                "headset_frame": LaunchConfiguration("headset_frame"),
                "left_vr_hand_frame": LaunchConfiguration("left_vr_hand_frame"),
                "right_vr_hand_frame": LaunchConfiguration("right_vr_hand_frame"),
                "left_base_frame": LaunchConfiguration("left_base_frame"),
                "right_base_frame": LaunchConfiguration("right_base_frame"),
                "left_ee_frame": LaunchConfiguration("left_ee_frame"),
                "right_ee_frame": LaunchConfiguration("right_ee_frame"),
                "joy_topic": LaunchConfiguration("joy_topic"),
                "output_file": LaunchConfiguration("output_file"),
                "sample_duration": LaunchConfiguration("sample_duration"),
                "sample_rate": LaunchConfiguration("sample_rate"),
                "mirror_convergence": LaunchConfiguration("mirror_convergence"),
            }],
        ),
    ])
