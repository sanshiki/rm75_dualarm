# ================================================================
# Mouse teleoperation — absolute screen-to-plane mapping
# ================================================================
# Maps the mouse position on screen directly to a position on a
# fixed 3-D plane.  Orientation is fixed.  Hold LEFT button to
# activate, release to stop.
#
# Usage:
#   ros2 launch rm_dualarm mouse_teleop.launch.py
#   ros2 launch rm_dualarm mouse_teleop.launch.py fixed_x:=0.40 y_min:=-0.3 y_max:=0.3
# ================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Fixed X distance from robot base [m]
    ld.add_action(DeclareLaunchArgument("fixed_x", default_value="0.35"))
    # Y range [m] (screen left → right  =  robot left → right)
    ld.add_action(DeclareLaunchArgument("y_min", default_value="-0.35"))
    ld.add_action(DeclareLaunchArgument("y_max", default_value="0.35"))
    # Z range [m] (screen bottom → top  =  robot low → high)
    ld.add_action(DeclareLaunchArgument("z_min", default_value="0.15"))
    ld.add_action(DeclareLaunchArgument("z_max", default_value="0.70"))
    # Fixed orientation
    ld.add_action(DeclareLaunchArgument("fixed_qx", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("fixed_qy", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("fixed_qz", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("fixed_qw", default_value="1.0"))
    # Rate & topic
    ld.add_action(DeclareLaunchArgument("publish_rate", default_value="30.0"))
    ld.add_action(DeclareLaunchArgument("target_topic", default_value="/target_pose"))
    ld.add_action(DeclareLaunchArgument("base_frame", default_value="base_link"))

    mouse_teleop_node = Node(
        package="rm_dualarm",
        executable="mouse_teleop_node.py",
        name="mouse_teleop",
        output="screen",
        parameters=[{
            "fixed_x": LaunchConfiguration("fixed_x"),
            "y_min": LaunchConfiguration("y_min"),
            "y_max": LaunchConfiguration("y_max"),
            "z_min": LaunchConfiguration("z_min"),
            "z_max": LaunchConfiguration("z_max"),
            "fixed_qx": LaunchConfiguration("fixed_qx"),
            "fixed_qy": LaunchConfiguration("fixed_qy"),
            "fixed_qz": LaunchConfiguration("fixed_qz"),
            "fixed_qw": LaunchConfiguration("fixed_qw"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "target_topic": LaunchConfiguration("target_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
        }],
    )
    ld.add_action(mouse_teleop_node)
    return ld
