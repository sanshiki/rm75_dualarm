# ================================================================
# VR teleoperation — full pipeline
# ================================================================
# Data flow:
#   Quest VR → ROS-TCP-Endpoint → /quest/joystick + TF (headset/hand)
#   → vr_base_broadcaster (headset TF → vr_base frame)
#   → vr_teleop_node (hand TF → /target_pose)
#   → servo_pose_tracking_demo → robot
#
# Usage:
#   ros2 launch rm_dualarm vr_teleop.launch.py
# ================================================================

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_node_params(params_file, node_name):
    with open(params_file, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f) or {}
    return params.get(node_name, {}).get("ros__parameters", {})


def _launch_default(params, name, fallback):
    value = params.get(name, fallback)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory("rm_dualarm")
    params_file = os.path.join(pkg_share, "config", "vr_teleop_params.yaml")
    base_params = _load_node_params(params_file, "vr_base_broadcaster")
    teleop_params = _load_node_params(params_file, "vr_teleop")

    # ---- vr_base_broadcaster args ----
    ld.add_action(DeclareLaunchArgument(
        "odom_frame",
        default_value=_launch_default(base_params, "odom_frame", "odom"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "headset_frame",
        default_value=_launch_default(base_params, "headset_frame", "headset"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_headset_yaw",
        default_value=_launch_default(base_params, "use_headset_yaw", False),
    ))
    ld.add_action(DeclareLaunchArgument(
        "broadcast_rate",
        default_value=_launch_default(base_params, "rate", 30.0),
    ))

    # ---- vr_teleop args ----
    ld.add_action(DeclareLaunchArgument(
        "control_mode",
        default_value=_launch_default(teleop_params, "control_mode", "single"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "target_topic",
        default_value=_launch_default(teleop_params, "target_topic", "/target_pose"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "left_target_topic",
        default_value=_launch_default(teleop_params, "left_target_topic", "/left/target_pose"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "right_target_topic",
        default_value=_launch_default(teleop_params, "right_target_topic", "/right/target_pose"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "active_topic",
        default_value=_launch_default(teleop_params, "active_topic", "/teleop_active"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "left_active_topic",
        default_value=_launch_default(teleop_params, "left_active_topic", "/left/teleop_active"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "right_active_topic",
        default_value=_launch_default(
            teleop_params, "right_active_topic", "/right/teleop_active"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "left_gripper_topic",
        default_value=_launch_default(teleop_params, "left_gripper_topic", "/left/gripper_cmd"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "right_gripper_topic",
        default_value=_launch_default(teleop_params, "right_gripper_topic", "/right/gripper_cmd"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "gripper_driver_topic",
        default_value=_launch_default(teleop_params, "gripper_driver_topic",
                                      "/rm_driver/set_gripper_position_cmd"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "joy_topic",
        default_value=_launch_default(teleop_params, "joy_topic", "/quest/joystick"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "vr_base_frame",
        default_value=_launch_default(teleop_params, "vr_base_frame", "vr_base"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "vr_origin_frame",
        default_value=_launch_default(teleop_params, "vr_origin_frame", "vr_origin"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "vr_hand_frame",
        default_value=_launch_default(teleop_params, "vr_hand_frame", "hand_right"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "left_vr_hand_frame",
        default_value=_launch_default(teleop_params, "left_vr_hand_frame", "hand_left"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "right_vr_hand_frame",
        default_value=_launch_default(teleop_params, "right_vr_hand_frame", "hand_right"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "base_frame",
        default_value=_launch_default(teleop_params, "base_frame", "base_link"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "left_base_frame",
        default_value=_launch_default(teleop_params, "left_base_frame", "left_base_link"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "right_base_frame",
        default_value=_launch_default(teleop_params, "right_base_frame", "right_base_link"),
    ))
    ld.add_action(DeclareLaunchArgument(
        "publish_rate",
        default_value=_launch_default(teleop_params, "publish_rate", 50.0),
    ))
    ld.add_action(DeclareLaunchArgument(
        "p_sensitivity",
        default_value=_launch_default(teleop_params, "p_sensitivity", 4.0),
    ))
    ld.add_action(DeclareLaunchArgument(
        "q_sensitivity",
        default_value=_launch_default(teleop_params, "q_sensitivity", 4.0),
    ))
    ld.add_action(DeclareLaunchArgument(
        "user_height",
        default_value=_launch_default(teleop_params, "user_height", 1.75),
    ))
    ld.add_action(DeclareLaunchArgument(
        "dual_y_offset",
        default_value=_launch_default(teleop_params, "dual_y_offset", 0.0),
    ))
    ld.add_action(DeclareLaunchArgument(
        "calibration_enabled",
        default_value=_launch_default(teleop_params, "calibration_enabled", True),
    ))
    ld.add_action(DeclareLaunchArgument(
        "calibration_file",
        default_value=_launch_default(teleop_params, "calibration_file", ""),
    ))
    ld.add_action(DeclareLaunchArgument(
        "mirror_mode",
        default_value=_launch_default(teleop_params, "mirror_mode", False),
    ))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    # ================================================================
    # 1. Relay receiver (TCP JSON from Docker ROS 1 sender)
    #    Run relay_sender.py inside the Docker container alongside the
    #    ROS 1 ros_tcp_endpoint:
    #      docker exec aubo-ros python3 relay_sender.py --host <HOST_IP> --port 7654
    # ================================================================
    ld.add_action(DeclareLaunchArgument("bind_ip", default_value="172.17.0.1"))
    ld.add_action(DeclareLaunchArgument("relay_port", default_value="7654"))
    ld.add_action(DeclareLaunchArgument("use_relay_receiver", default_value="true"))
    relay_rx = Node(
        package="rm_dualarm",
        executable="relay_receiver.py",
        name="relay_receiver",
        output="screen",
        parameters=[{
            "bind_ip": LaunchConfiguration("bind_ip"),
            "port": LaunchConfiguration("relay_port"),
        }],
        condition=IfCondition(LaunchConfiguration("use_relay_receiver")),
    )
    ld.add_action(relay_rx)

    # ================================================================
    # 2. VR base broadcaster (headset TF → vr_base frame)
    # ================================================================
    # vr_base_br = Node(
    #     package="rm_dualarm",
    #     executable="vr_base_broadcaster.py",
    #     name="vr_base_broadcaster",
    #     output="screen",
    #     parameters=[{
    #         "odom_frame": LaunchConfiguration("odom_frame"),
    #         "headset_frame": LaunchConfiguration("headset_frame"),
    #         "use_headset_yaw": LaunchConfiguration("use_headset_yaw"),
    #         "rate": LaunchConfiguration("broadcast_rate"),
    #     }],
    # )
    # ld.add_action(vr_base_br)

    # ================================================================
    # 3. VR teleop node (hand TF → /target_pose)
    # ================================================================
    vr_node = Node(
        package="rm_dualarm",
        executable="vr_teleop_node.py",
        name="vr_teleop",
        output="screen",
        parameters=[
            params_file,
            {
                "control_mode": LaunchConfiguration("control_mode"),
                "target_topic": LaunchConfiguration("target_topic"),
                "left_target_topic": LaunchConfiguration("left_target_topic"),
                "right_target_topic": LaunchConfiguration("right_target_topic"),
                "active_topic": LaunchConfiguration("active_topic"),
                "left_active_topic": LaunchConfiguration("left_active_topic"),
                "right_active_topic": LaunchConfiguration("right_active_topic"),
                "left_gripper_topic": LaunchConfiguration("left_gripper_topic"),
                "right_gripper_topic": LaunchConfiguration("right_gripper_topic"),
                "gripper_driver_topic": LaunchConfiguration("gripper_driver_topic"),
                "joy_topic": LaunchConfiguration("joy_topic"),
                "vr_base_frame": LaunchConfiguration("vr_base_frame"),
                "vr_origin_frame": LaunchConfiguration("vr_origin_frame"),
                "vr_hand_frame": LaunchConfiguration("vr_hand_frame"),
                "left_vr_hand_frame": LaunchConfiguration("left_vr_hand_frame"),
                "right_vr_hand_frame": LaunchConfiguration("right_vr_hand_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "left_base_frame": LaunchConfiguration("left_base_frame"),
                "right_base_frame": LaunchConfiguration("right_base_frame"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "p_sensitivity": LaunchConfiguration("p_sensitivity"),
                "q_sensitivity": LaunchConfiguration("q_sensitivity"),
                "user_height": LaunchConfiguration("user_height"),
                "dual_y_offset": LaunchConfiguration("dual_y_offset"),
                "calibration_enabled": LaunchConfiguration("calibration_enabled"),
                "calibration_file": LaunchConfiguration("calibration_file"),
                "mirror_mode": LaunchConfiguration("mirror_mode"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )
    ld.add_action(vr_node)

    return ld
