# ================================================================
# Dual-arm real robot bringup
# ================================================================
# Starts TWO rm_driver instances (one per physical arm, each with its
# own IP), merges their joint_states into a single prefixed stream,
# and launches the dual-arm robot_state_publisher and optionally
# move_group + pose_init.
#
# Usage:
#   ros2 launch rm_dualarm dual_real_bringup.launch.py \
#       left_arm_ip:=192.168.1.18 right_arm_ip:=192.168.1.19
#
# Then start servo in a second terminal:
#   ros2 launch rm_dualarm servo_real.launch.py control_mode:=dual
# ================================================================

import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from dual_moveit_utils import (  # noqa: E402
    dual_real_moveit_config,
    resolve_dual_arm_layout,
)


def generate_launch_description():
    ld = LaunchDescription()

    for name, default in (
        ("left_arm_ip", "192.168.1.18"),
        ("right_arm_ip", "192.168.1.19"),
        ("init_delay", "6.0"),
        ("use_pose_init", "false"),
        ("use_move_group", "true"),
        ("standby_pose_file", ""),
        ("base_x", ""),
        ("base_z", ""),
        ("base_spacing_y", ""),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    def setup(context):
        pkg_share = get_package_share_directory("rm_dualarm")

        left_ip = LaunchConfiguration("left_arm_ip").perform(context)
        right_ip = LaunchConfiguration("right_arm_ip").perform(context)

        # ---- Build per-arm driver parameter dicts --------------------
        base_config_path = os.path.join(
            get_package_share_directory("rm_driver"),
            "config",
            "rm_75_config.yaml",
        )
        with open(base_config_path, "r") as f:
            base_cfg = yaml.safe_load(f)
        template_params = base_cfg["rm_driver"]["ros__parameters"]

        left_params = dict(template_params)
        left_params["arm_ip"] = left_ip
        # Keep default udp_port 8089 for left arm

        right_params = dict(template_params)
        right_params["arm_ip"] = right_ip
        right_params["udp_port"] = 8090  # avoid conflict with left arm

        # ---- Dual-arm URDF & moveit config ---------------------------
        layout = resolve_dual_arm_layout(pkg_share, {
            "base_x": LaunchConfiguration("base_x").perform(context),
            "base_z": LaunchConfiguration("base_z").perform(context),
            "base_spacing_y": LaunchConfiguration("base_spacing_y").perform(context),
        })
        moveit_config = dual_real_moveit_config(pkg_share, layout)
        robot_description = moveit_config["robot_description"]

        actions = []

        # ---- Left rm_driver (namespace /left) ------------------------
        actions.append(Node(
            package="rm_driver",
            executable="rm_driver",
            namespace="/left",
            name="rm_driver",
            output="screen",
            parameters=[left_params],
        ))

        # ---- Right rm_driver (namespace /right) ----------------------
        actions.append(Node(
            package="rm_driver",
            executable="rm_driver",
            namespace="/right",
            name="rm_driver",
            output="screen",
            parameters=[right_params],
        ))

        # ---- Joint state merger --------------------------------------
        # Merges /left/joint_states + /right/joint_states → /joint_states
        # with left_/right_ prefixes matching the dual URDF.
        actions.append(Node(
            package="rm_dualarm",
            executable="dual_joint_state_merger.py",
            name="dual_joint_state_merger",
            output="screen",
        ))

        # ---- Robot state publisher (dual real URDF) ------------------
        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ))

        # ---- MoveGroup (dual config) ---------------------------------
        move_group_params = [
            moveit_config,
            {
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
                "use_sim_time": False,
            },
        ]
        actions.append(Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=move_group_params,
            additional_env={"DISPLAY": ":0"},
            condition=IfCondition(LaunchConfiguration("use_move_group")),
        ))

        # ---- Pose init (dual, move-to-standby) -----------------------
        actions.append(Node(
            package="rm_dualarm",
            executable="pose_init.py",
            name="pose_init",
            output="screen",
            parameters=[{
                "delay_seconds": LaunchConfiguration("init_delay"),
                "control_mode": "dual",
                "standby_pose_file": LaunchConfiguration("standby_pose_file"),
                "use_sim_time": False,
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_pose_init"), "' == 'true' and '",
                LaunchConfiguration("use_move_group"), "' == 'true'",
            ])),
        ))

        return actions

    ld.add_action(OpaqueFunction(function=setup))
    return ld
