# ================================================================
# MoveIt2 Servo — Standalone launch for real RM75 robot
# ================================================================
# Starts ONLY servo-related nodes.  You must launch the driver separately:
#
#   ros2 launch rm_driver rm_75_driver.launch.py arm_ip:=192.168.1.18
#   ros2 launch rm_dualarm servo_real.launch.py
#
# Data flow:
#   mouse_teleop → PoseStamped → servo_pose_tracking_demo (PID → Servo)
#   → JointTrajectory → servo_bridge (50Hz) → Jointpos CANFD → rm_driver
#
# References:
#   /opt/ros/humble/share/moveit_servo/launch/pose_tracking_example.launch.py
#   https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html
# ================================================================

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory("rm_dualarm")

    # ---- Launch arguments ----
    ld.add_action(
        DeclareLaunchArgument(
            "use_pose_tracking", default_value="true",
            description="true=servo_pose_tracking_demo (PoseStamped), "
                        "false=servo_node_main (TwistStamped/JointJog)"
        )
    )
    ld.add_action(
        DeclareLaunchArgument("use_mouse_teleop", default_value="true")
    )
    ld.add_action(
        DeclareLaunchArgument("fixed_x", default_value="0.35")
    )
    ld.add_action(
        DeclareLaunchArgument("y_min", default_value="-0.35")
    )
    ld.add_action(
        DeclareLaunchArgument("y_max", default_value="0.35")
    )
    ld.add_action(
        DeclareLaunchArgument("use_rviz", default_value="true",
                             description="Launch RViz with MoveIt config")
    )

    use_pose_tracking = LaunchConfiguration("use_pose_tracking")
    use_mouse_teleop = LaunchConfiguration("use_mouse_teleop")
    use_rviz = LaunchConfiguration("use_rviz")

    # ---- MoveIt2 configuration ----
    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .to_moveit_configs()
    )

    # ---- 1. Robot State Publisher (URDF → TF) ----
    # The real driver does not publish the URDF or transforms, so we
    # need an rsp node to provide them for MoveIt2 and Servo.
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    ld.add_action(rsp_node)

    # ---- 2. move_group (planning scene monitor for Servo) ----
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
    )
    ld.add_action(move_group_node)

    # ---- 3. MoveIt2 Servo ----
    servo_real_cfg = yaml.safe_load(
        open(os.path.join(pkg_share, "config", "servo_75_real.yaml"))
    )
    pose_tracking_cfg = yaml.safe_load(
        open(os.path.join(pkg_share, "config", "pose_tracking_settings.yaml"))
    )

    servo_params = {
        "moveit_servo": servo_real_cfg,
    }

    servo_params_pt = {
        "moveit_servo": {**servo_real_cfg, **pose_tracking_cfg},
    }

    # ---- 3a. servo_pose_tracking_demo (PoseStamped → PID → Servo) ----
    pose_tracking_node = Node(
        package="moveit_servo",
        executable="servo_pose_tracking_demo",
        name="servo_pose_tracking",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params_pt,
        ],
        condition=IfCondition(use_pose_tracking),
    )
    ld.add_action(pose_tracking_node)

    # ---- 3b. servo_node_main (TwistStamped / JointJog input) ----
    # After launch, start servo with:
    #   ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        condition=UnlessCondition(use_pose_tracking),
    )
    ld.add_action(servo_node)

    # ---- 4. Servo bridge (JointTrajectory → Jointpos CANFD) ----
    servo_bridge_node = Node(
        package="rm_dualarm",
        executable="servo_bridge",
        name="servo_bridge",
        output="screen",
        parameters=[
            {
                "arm_dof": 7,
                "follow_mode": True,
                "publish_rate": 50.0,
                "command_timeout": 0.15,
                "halt_on_timeout": True,
                "servo_input_topic": "/servo_bridge/joint_trajectory_in",
                "driver_topic": "/rm_driver/movej_canfd_cmd",
            }
        ],
    )
    ld.add_action(servo_bridge_node)

    # ---- 5. (Optional) RViz ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("rm_75_config"),
                "config",
                "moveit.rviz",
            ),
        ],
        parameters=[
            moveit_config.to_dict(),
        ],
        condition=IfCondition(use_rviz),
    )
    ld.add_action(rviz_node)

    # ---- 6. (Optional) Mouse teleop ----
    mouse_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "mouse_teleop.launch.py")
        ),
        condition=IfCondition(use_mouse_teleop),
        launch_arguments={
            "fixed_x": LaunchConfiguration("fixed_x"),
            "y_min": LaunchConfiguration("y_min"),
            "y_max": LaunchConfiguration("y_max"),
        }.items(),
    )
    ld.add_action(mouse_teleop_launch)

    return ld
