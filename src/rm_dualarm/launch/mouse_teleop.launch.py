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
    ld.add_action(DeclareLaunchArgument("static_quat", default_value="true"))
    # Rate & topic
    ld.add_action(DeclareLaunchArgument("publish_rate", default_value="50.0"))
    ld.add_action(DeclareLaunchArgument("control_mode", default_value="single"))
    ld.add_action(DeclareLaunchArgument("target_topic", default_value="/target_pose"))
    ld.add_action(DeclareLaunchArgument("left_target_topic", default_value="/left/target_pose"))
    ld.add_action(DeclareLaunchArgument("right_target_topic", default_value="/right/target_pose"))
    ld.add_action(DeclareLaunchArgument("active_topic", default_value="/teleop_active"))
    ld.add_action(DeclareLaunchArgument("left_active_topic", default_value="/left/teleop_active"))
    ld.add_action(DeclareLaunchArgument(
        "right_active_topic", default_value="/right/teleop_active"))
    ld.add_action(DeclareLaunchArgument("base_frame", default_value="base_link"))
    ld.add_action(DeclareLaunchArgument("left_base_frame", default_value="left_base_link"))
    ld.add_action(DeclareLaunchArgument("right_base_frame", default_value="right_base_link"))
    ld.add_action(DeclareLaunchArgument("ee_frame", default_value="Link7"))
    ld.add_action(DeclareLaunchArgument("left_ee_frame", default_value="left_Link7"))
    ld.add_action(DeclareLaunchArgument("right_ee_frame", default_value="right_Link7"))
    ld.add_action(DeclareLaunchArgument("dual_y_offset", default_value="0.25"))

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
            "static_quat": LaunchConfiguration("static_quat"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "control_mode": LaunchConfiguration("control_mode"),
            "target_topic": LaunchConfiguration("target_topic"),
            "left_target_topic": LaunchConfiguration("left_target_topic"),
            "right_target_topic": LaunchConfiguration("right_target_topic"),
            "active_topic": LaunchConfiguration("active_topic"),
            "left_active_topic": LaunchConfiguration("left_active_topic"),
            "right_active_topic": LaunchConfiguration("right_active_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "left_base_frame": LaunchConfiguration("left_base_frame"),
            "right_base_frame": LaunchConfiguration("right_base_frame"),
            "ee_frame": LaunchConfiguration("ee_frame"),
            "left_ee_frame": LaunchConfiguration("left_ee_frame"),
            "right_ee_frame": LaunchConfiguration("right_ee_frame"),
            "dual_y_offset": LaunchConfiguration("dual_y_offset"),
        }],
    )
    ld.add_action(mouse_teleop_node)
    return ld
