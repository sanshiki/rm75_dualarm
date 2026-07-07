# ================================================================
# USB/V4L2 camera bringup
# ================================================================
# Publishes sensor_msgs/Image and CameraInfo from a local V4L2 device.
#
# Usage:
#   ros2 launch rm_dualarm camera.launch.py
#   ros2 launch rm_dualarm camera.launch.py video_device:=/dev/video2 image_width:=1280 image_height:=720
#   ros2 launch rm_dualarm camera.launch.py camera_namespace:=left_camera camera_frame_id:=left_camera_optical_frame
# ================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_node(context, *args, **kwargs):
    image_width = int(LaunchConfiguration("image_width").perform(context))
    image_height = int(LaunchConfiguration("image_height").perform(context))

    return [
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name=LaunchConfiguration("camera_name"),
            namespace=LaunchConfiguration("camera_namespace"),
            output="screen",
            parameters=[{
                "video_device": LaunchConfiguration("video_device"),
                "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
                "pixel_format": LaunchConfiguration("pixel_format"),
                "output_encoding": LaunchConfiguration("output_encoding"),
                "image_size": [image_width, image_height],
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        )
    ]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("video_device", default_value="/dev/v4l/by-id/usb-SN0002_2K_USB_Camera_46435000_P020300_SN0002-video-index0"))
    ld.add_action(DeclareLaunchArgument("camera_name", default_value="camera"))
    ld.add_action(DeclareLaunchArgument("camera_namespace", default_value="camera"))
    ld.add_action(DeclareLaunchArgument("camera_frame_id", default_value="camera_optical_frame"))
    ld.add_action(DeclareLaunchArgument("camera_info_url", default_value=""))
    ld.add_action(DeclareLaunchArgument("image_width", default_value="640"))
    ld.add_action(DeclareLaunchArgument("image_height", default_value="480"))
    ld.add_action(DeclareLaunchArgument("pixel_format", default_value="YUYV"))
    ld.add_action(DeclareLaunchArgument("output_encoding", default_value="rgb8"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    ld.add_action(OpaqueFunction(function=_camera_node))
    return ld
