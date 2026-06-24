import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from dual_moveit_utils import dual_moveit_config, resolve_dual_arm_layout  # noqa: E402


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("use_gazebo", default_value="true"))
    ld.add_action(DeclareLaunchArgument("gazebo_gui", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_spawn", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_move_group", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_pose_init", default_value="true"))
    ld.add_action(DeclareLaunchArgument("init_delay", default_value="8.0"))
    ld.add_action(DeclareLaunchArgument("standby_pose_file", default_value=""))
    ld.add_action(DeclareLaunchArgument("base_x", default_value=""))
    ld.add_action(DeclareLaunchArgument("base_z", default_value=""))
    ld.add_action(DeclareLaunchArgument("base_spacing_y", default_value=""))

    def setup(context, *args, **kwargs):
        pkg_share = get_package_share_directory("rm_dualarm")
        layout = resolve_dual_arm_layout(pkg_share, {
            "base_x": LaunchConfiguration("base_x").perform(context),
            "base_z": LaunchConfiguration("base_z").perform(context),
            "base_spacing_y": LaunchConfiguration("base_spacing_y").perform(context),
        })
        moveit_config = dual_moveit_config(pkg_share, layout)
        robot_description = moveit_config["robot_description"]
        actions = []

        actions.append(ExecuteProcess(
            cmd=[
                "gazebo", "--verbose",
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
            ],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_gazebo"), "' == 'true' and '",
                LaunchConfiguration("gazebo_gui"), "' == 'true'",
            ])),
        ))
        actions.append(ExecuteProcess(
            cmd=[
                "gzserver", "--verbose",
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
            ],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_gazebo"), "' == 'true' and '",
                LaunchConfiguration("gazebo_gui"), "' == 'false'",
            ])),
        ))

        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"use_sim_time": True},
                {"robot_description": robot_description},
                {"publish_frequency": 30.0},
            ],
            output="screen",
        ))

        spawn_entity = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "rm_75_dualarm"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_spawn")),
        )
        actions.append(spawn_entity)

        load_joint_state = ExecuteProcess(
            cmd=[
                "ros2", "control", "load_controller",
                "--set-state", "active", "joint_state_broadcaster",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_spawn")),
        )
        load_left_controller = ExecuteProcess(
            cmd=[
                "ros2", "control", "load_controller",
                "--set-state", "active", "left_rm_group_controller",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_spawn")),
        )
        load_right_controller = ExecuteProcess(
            cmd=[
                "ros2", "control", "load_controller",
                "--set-state", "active", "right_rm_group_controller",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_spawn")),
        )

        actions.append(RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state])
        ))
        actions.append(RegisterEventHandler(
            event_handler=OnProcessExit(target_action=load_joint_state, on_exit=[load_left_controller])
        ))
        actions.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_left_controller,
                on_exit=[load_right_controller],
            )
        ))

        actions.append(Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                moveit_config,
                {
                    "publish_robot_description_semantic": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "monitor_dynamics": False,
                    "use_sim_time": True,
                },
            ],
            condition=IfCondition(LaunchConfiguration("use_move_group")),
        ))

        pose_init_node = Node(
            package="rm_dualarm",
            executable="pose_init.py",
            name="dual_pose_init",
            output="screen",
            parameters=[{
                "delay_seconds": LaunchConfiguration("init_delay"),
                "control_mode": "dual",
                "standby_pose_file": LaunchConfiguration("standby_pose_file"),
                "use_sim_time": True,
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_pose_init"), "' == 'true' and '",
                LaunchConfiguration("use_move_group"), "' == 'true'",
            ])),
        )
        actions.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_right_controller,
                on_exit=[pose_init_node],
            )
        ))

        return actions

    ld.add_action(OpaqueFunction(function=setup))

    return ld
