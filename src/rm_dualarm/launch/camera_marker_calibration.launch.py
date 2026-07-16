# ================================================================
# AprilTag/ArUco camera pose capture and drift checker
# ================================================================
# Usage:
#   ros2 launch rm_dualarm camera_marker_calibration.launch.py mode:=capture marker_size_m:=0.063
#   ros2 launch rm_dualarm camera_marker_calibration.launch.py mode:=check
# ================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("rm_dualarm")
    default_params = os.path.join(
        pkg_share, "config", "camera_marker_calibration.yaml")
    default_reference = os.path.join(
        pkg_share, "config", "camera_marker_reference.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("mode", default_value="check"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("marker_dictionary", default_value="DICT_APRILTAG_36h11"),
        DeclareLaunchArgument("marker_id", default_value="0"),
        DeclareLaunchArgument("marker_size_m", default_value="0.095"),
        DeclareLaunchArgument("marker_frame", default_value="calibration_marker"),
        DeclareLaunchArgument("camera_frame", default_value=""),
        DeclareLaunchArgument("approximate_horizontal_fov_deg", default_value="60.0"),
        DeclareLaunchArgument("output_file", default_value=default_reference),
        DeclareLaunchArgument("reference_file", default_value=default_reference),
        DeclareLaunchArgument("sample_count", default_value="50"),
        DeclareLaunchArgument("min_valid_samples", default_value="20"),
        DeclareLaunchArgument("status_period", default_value="1.0"),
        DeclareLaunchArgument("translation_warn_mm", default_value="10.0"),
        DeclareLaunchArgument("rotation_warn_deg", default_value="2.0"),
        DeclareLaunchArgument("log_check_status", default_value="true"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        DeclareLaunchArgument("exit_after_capture", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="rm_dualarm",
            executable="camera_marker_calibration_node.py",
            name="camera_marker_calibration",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "mode": LaunchConfiguration("mode"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "marker_dictionary": LaunchConfiguration("marker_dictionary"),
                    "marker_id": ParameterValue(LaunchConfiguration("marker_id"), value_type=int),
                    "marker_size_m": ParameterValue(LaunchConfiguration("marker_size_m"), value_type=float),
                    "marker_frame": LaunchConfiguration("marker_frame"),
                    "camera_frame": LaunchConfiguration("camera_frame"),
                    "approximate_horizontal_fov_deg": ParameterValue(
                        LaunchConfiguration("approximate_horizontal_fov_deg"),
                        value_type=float),
                    "output_file": LaunchConfiguration("output_file"),
                    "reference_file": LaunchConfiguration("reference_file"),
                    "sample_count": ParameterValue(LaunchConfiguration("sample_count"), value_type=int),
                    "min_valid_samples": ParameterValue(LaunchConfiguration("min_valid_samples"), value_type=int),
                    "status_period": ParameterValue(LaunchConfiguration("status_period"), value_type=float),
                    "translation_warn_mm": ParameterValue(LaunchConfiguration("translation_warn_mm"), value_type=float),
                    "rotation_warn_deg": ParameterValue(LaunchConfiguration("rotation_warn_deg"), value_type=float),
                    "log_check_status": ParameterValue(LaunchConfiguration("log_check_status"), value_type=bool),
                    "publish_tf": ParameterValue(LaunchConfiguration("publish_tf"), value_type=bool),
                    "exit_after_capture": ParameterValue(LaunchConfiguration("exit_after_capture"), value_type=bool),
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
