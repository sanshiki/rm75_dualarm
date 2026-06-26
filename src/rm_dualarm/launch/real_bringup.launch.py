# ================================================================
# Real robot bringup — rm_driver + rm_control + move_group + pose_init
# ================================================================
# Launches the rm_driver (TCP communication with the arm), rm_control
# (FollowJointTrajectory action server), robot_state_publisher,
# and optionally MoveIt move_group + pose_init.
#
# Usage:
#   ros2 launch rm_dualarm real_bringup.launch.py arm_ip:=192.168.1.18
# ================================================================

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("arm_ip", default_value="192.168.1.18"))
    ld.add_action(DeclareLaunchArgument("init_delay", default_value="6.0"))
    ld.add_action(DeclareLaunchArgument("use_pose_init", default_value="false"))
    ld.add_action(DeclareLaunchArgument("standby_pose_file", default_value=""))
    ld.add_action(DeclareLaunchArgument("use_move_group", default_value="true"))

    # ------------------------------------------------------------------
    # rm_driver — loaded from the template YAML with arm_ip overridden
    # ------------------------------------------------------------------
    base_config_path = os.path.join(
        get_package_share_directory("rm_driver"),
        "config",
        "rm_75_config.yaml",
    )
    with open(base_config_path, "r") as f:
        base_cfg = yaml.safe_load(f)
    driver_params = dict(base_cfg["rm_driver"]["ros__parameters"])
    driver_params["arm_ip"] = LaunchConfiguration("arm_ip")

    rm_driver_node = Node(
        package="rm_driver",
        executable="rm_driver",
        name="rm_driver",
        output="screen",
        parameters=[driver_params],
    )
    ld.add_action(rm_driver_node)

    # ------------------------------------------------------------------
    # rm_control — FollowJointTrajectory action server for move_group
    # ------------------------------------------------------------------
    rm_control_node = Node(
        package="rm_control",
        executable="rm_control",
        name="rm_control",
        output="screen",
        parameters=[
            {"follow": True},
            {"arm_type": 75},
        ],
    )
    ld.add_action(rm_control_node)

    # ------------------------------------------------------------------
    # MoveIt & robot_state_publisher
    # ------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .to_moveit_configs()
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    ld.add_action(rsp_node)

    move_group_params = [
        moveit_config.to_dict(),
        {
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
        },
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": ":0"},
        condition=IfCondition(LaunchConfiguration("use_move_group")),
    )
    ld.add_action(move_group_node)

    # ------------------------------------------------------------------
    # pose_init — move arm to standby pose (requires move_group)
    # ------------------------------------------------------------------
    pose_init_node = Node(
        package="rm_dualarm",
        executable="pose_init.py",
        name="pose_init",
        output="screen",
        parameters=[{
            "delay_seconds": LaunchConfiguration("init_delay"),
            "standby_pose_file": LaunchConfiguration("standby_pose_file"),
            "use_sim_time": False,
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("use_pose_init"), "' == 'true' and '",
            LaunchConfiguration("use_move_group"), "' == 'true'",
        ])),
    )
    ld.add_action(pose_init_node)

    return ld
