# ================================================================
# MoveIt2 Servo — Standalone launch for Gazebo simulation
# ================================================================
# Starts ONLY servo-related nodes.  You must launch Gazebo separately:
#
#   ros2 launch rm_gazebo gazebo_75_demo.launch.py
#   ros2 launch rm_dualarm servo_sim.launch.py
#
# Servo publishes JointTrajectory directly to the
# rm_group_controller in Gazebo — no servo_bridge needed in sim.
#
# References:
#   /opt/ros/humble/share/moveit_servo/launch/pose_tracking_example.launch.py
#   https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html
# ================================================================

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory("rm_dualarm")

    # ---- Launch arguments ----
    ld.add_action(
        DeclareLaunchArgument(
            "use_pose_tracking", default_value="false",
            description="true=servo_pose_tracking_demo (PoseStamped), "
                        "false=servo_node_main (TwistStamped/JointJog) [default]"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "teleop_type", default_value="mouse",
            description="mouse | vr | none"
        )
    )
    ld.add_action(
        DeclareLaunchArgument("fixed_x", default_value="0.25")
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
    teleop_type = LaunchConfiguration("teleop_type")
    use_rviz = LaunchConfiguration("use_rviz")

    # ---- MoveIt2 configuration (URDF + SRDF + kinematics) ----
    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .to_moveit_configs()
    )

    # ---- 1. MoveIt2 Servo ----
    # Load YAML configs with yaml.safe_load() and nest under the
    # 'moveit_servo' namespace as expected by servo_node_main /
    # servo_pose_tracking_demo.
    servo_sim_cfg = yaml.safe_load(
        open(os.path.join(pkg_share, "config", "servo_75_sim.yaml"))
    )
    pose_tracking_cfg = yaml.safe_load(
        open(os.path.join(pkg_share, "config", "pose_tracking_settings.yaml"))
    )

    servo_params = {
        "moveit_servo": servo_sim_cfg,
        "use_sim_time": True,
    }

    servo_params_pt = {
        "moveit_servo": {**servo_sim_cfg, **pose_tracking_cfg},
        "use_sim_time": True,
    }

    # ---- 1a. servo_pose_tracking_demo (PoseStamped → PID → Servo) ----
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

    # ---- 1b. servo_node_main (TwistStamped / JointJog → Servo) ----
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

    # Auto-start the servo after a short delay
    start_servo = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "service", "call", "/servo_node/start_servo",
                     "std_srvs/srv/Trigger", "{}"],
                output="screen",
            )
        ],
        condition=UnlessCondition(use_pose_tracking),
    )
    ld.add_action(start_servo)

    # ---- 2. Trajectory relay (teleop_active → settle → forward) ----
    # traj_relay_node = Node(
    #     package="rm_dualarm",
    #     executable="trajectory_relay.py",
    #     name="trajectory_relay",
    #     output="screen",
    #     parameters=[{
    #         "input_topic": "/rm_group_controller/joint_trajectory_raw",
    #         "output_topic": "/rm_group_controller/joint_trajectory",
    #         "settle_seconds": 4.0,
    #         "max_wait": 3000.0,
    #         "use_sim_time": True,
    #     }],
    # )
    # ld.add_action(traj_relay_node)

    # ---- 3. PoseTracking node (PoseStamped → TwistStamped, shared) ----
    pose_tracking_node = Node(
        package="rm_dualarm",
        executable="pose_tracking_node.py",
        name="pose_tracking",
        output="screen",
        parameters=[
            # PID gains + filter settings (top-level, same YAML)
            {k: v for k, v in pose_tracking_cfg.items()
             if k in ("x_proportional_gain", "y_proportional_gain",
                      "z_proportional_gain", "x_integral_gain",
                      "y_integral_gain", "z_integral_gain",
                      "x_derivative_gain", "y_derivative_gain",
                      "z_derivative_gain", "angular_proportional_gain",
                      "angular_integral_gain", "angular_derivative_gain",
                      "windup_limit", "filter_enabled", "filter_alpha",
                      "safe_zone_enabled",
                      "x_min", "x_max", "y_min", "y_max", "z_min", "z_max")},
            {"use_sim_time": True},
        ],
    )
    ld.add_action(pose_tracking_node)

    # ---- 4. (Optional) RViz ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("rm_dualarm"),
                "rviz",
                "moveit.rviz",
            ),
        ],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
    )
    ld.add_action(rviz_node)

    # ---- 5. Teleop (mouse or VR, mutually exclusive) ----
    is_mouse = PythonExpression(["'", teleop_type, "' == 'mouse'"])
    is_vr    = PythonExpression(["'", teleop_type, "' == 'vr'"])

    mouse_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "mouse_teleop.launch.py")
        ),
        condition=IfCondition(is_mouse),
        launch_arguments={
            "fixed_x": LaunchConfiguration("fixed_x"),
            "y_min": LaunchConfiguration("y_min"),
            "y_max": LaunchConfiguration("y_max"),
        }.items(),
    )
    ld.add_action(mouse_teleop_launch)

    vr_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "vr_teleop.launch.py")
        ),
        condition=IfCondition(is_vr),
    )
    ld.add_action(vr_teleop_launch)

    return ld
