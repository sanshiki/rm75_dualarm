# ================================================================
# Simulation bringup — Gazebo + move_group + pose_init
# ================================================================
# Starts the full simulation environment and moves the arm to a
# non-singular pose ready for servo control.
#
# Usage:
#   ros2 launch rm_dualarm sim_bringup.launch.py
#   ros2 launch rm_dualarm sim_bringup.launch.py init_delay:=10.0
# ================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()
    control_mode = LaunchConfiguration("control_mode")

    # ---- Arguments ----
    ld.add_action(DeclareLaunchArgument(
        "init_delay",
        default_value="6.0",
        description="Delay [s] before pose_init connects",
    ))
    ld.add_action(
        DeclareLaunchArgument("use_pose_init", default_value="true")
    )
    ld.add_action(DeclareLaunchArgument("standby_pose_file", default_value=""))
    ld.add_action(DeclareLaunchArgument(
        "control_mode",
        default_value="single",
        description="single | dual",
    ))
    ld.add_action(DeclareLaunchArgument("use_gazebo", default_value="true"))
    ld.add_action(DeclareLaunchArgument("gazebo_gui", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_spawn", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_move_group", default_value="true"))
    ld.add_action(DeclareLaunchArgument("base_x", default_value=""))
    ld.add_action(DeclareLaunchArgument("base_z", default_value=""))
    ld.add_action(DeclareLaunchArgument("base_spacing_y", default_value=""))

    # ---- 1. Gazebo simulation (includes robot_state_publisher,
    #        spawn_entity, controller_manager, controllers) ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_gazebo"),
                "launch",
                "gazebo_75_demo.launch.py",
            )
        ),
        condition=IfCondition(PythonExpression(["'", control_mode, "' == 'single'"])),
    )
    ld.add_action(gazebo_launch)

    dual_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_dualarm"),
                "launch",
                "dual_sim_bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_gazebo": LaunchConfiguration("use_gazebo"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "use_spawn": LaunchConfiguration("use_spawn"),
            "use_move_group": LaunchConfiguration("use_move_group"),
            "use_pose_init": LaunchConfiguration("use_pose_init"),
            "init_delay": LaunchConfiguration("init_delay"),
            "base_x": LaunchConfiguration("base_x"),
            "base_z": LaunchConfiguration("base_z"),
            "base_spacing_y": LaunchConfiguration("base_spacing_y"),
        }.items(),
        condition=IfCondition(PythonExpression(["'", control_mode, "' == 'dual'"])),
    )
    ld.add_action(dual_gazebo_launch)

    # ---- 2. MoveIt2 configuration ----
    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .to_moveit_configs()
    )

    # ---- 3. move_group (planning scene + action server for pose_init) ----
    move_group_params = [
        moveit_config.to_dict(),
        {
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
            "use_sim_time": True,
        },
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": ":0"},
        condition=IfCondition(PythonExpression([
            "'", control_mode, "' == 'single' and '",
            LaunchConfiguration("use_move_group"), "' == 'true'",
        ])),
    )
    ld.add_action(move_group_node)

    # ---- 4. Pose initialisation (delayed internally) ----
    pose_init_node = Node(
        package="rm_dualarm",
        executable="pose_init.py",
        name="pose_init",
        output="screen",
        parameters=[{
            "delay_seconds": LaunchConfiguration("init_delay"),
            "standby_pose_file": LaunchConfiguration("standby_pose_file"),
            "use_sim_time": True,
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("use_pose_init"), "' == 'true' and '",
            control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(pose_init_node)

    return ld
