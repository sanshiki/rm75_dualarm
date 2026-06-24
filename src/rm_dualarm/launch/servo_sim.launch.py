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
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

sys.path.append(os.path.dirname(__file__))
from dual_moveit_utils import dual_moveit_config  # noqa: E402


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
        DeclareLaunchArgument(
            "control_mode", default_value="single",
            description="single | dual"
        )
    )
    ld.add_action(
        DeclareLaunchArgument("fixed_x", default_value="0.25")
    )
    ld.add_action(
        DeclareLaunchArgument("mouse_publish_rate", default_value="50.0")
    )
    ld.add_action(
        DeclareLaunchArgument("y_min", default_value="-0.35")
    )
    ld.add_action(
        DeclareLaunchArgument("y_max", default_value="0.35")
    )
    ld.add_action(DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz with MoveIt config",
    ))
    for name, default in (
        ("left_target_topic", "/left/target_pose"),
        ("right_target_topic", "/right/target_pose"),
        ("left_twist_topic", "/left_servo_node/delta_twist_cmds"),
        ("right_twist_topic", "/right_servo_node/delta_twist_cmds"),
        ("left_status_topic", "/left_servo_node/status"),
        ("right_status_topic", "/right_servo_node/status"),
        ("left_command_out_topic", "/left_rm_group_controller/joint_trajectory"),
        ("right_command_out_topic", "/right_rm_group_controller/joint_trajectory"),
        ("left_active_topic", "/left/teleop_active"),
        ("right_active_topic", "/right/teleop_active"),
        ("left_gripper_topic", "/left/gripper_cmd"),
        ("right_gripper_topic", "/right/gripper_cmd"),
        ("use_relay_receiver", "true"),
        ("left_base_frame", "left_base_link"),
        ("right_base_frame", "right_base_link"),
        ("left_ee_frame", "left_Link7"),
        ("right_ee_frame", "right_Link7"),
        ("left_move_group_name", "left_rm_group"),
        ("right_move_group_name", "right_rm_group"),
        ("left_servo_name", "left_servo_node"),
        ("right_servo_name", "right_servo_node"),
        ("vr_calibration_enabled", "true"),
        ("vr_calibration_file", ""),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    use_pose_tracking = LaunchConfiguration("use_pose_tracking")
    teleop_type = LaunchConfiguration("teleop_type")
    control_mode = LaunchConfiguration("control_mode")
    use_rviz = LaunchConfiguration("use_rviz")

    # ---- MoveIt2 configuration (URDF + SRDF + kinematics) ----
    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .to_moveit_configs()
    )
    single_moveit_config = moveit_config.to_dict()
    dual_moveit_config_dict = dual_moveit_config(pkg_share)

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

    left_servo_cfg = {**servo_sim_cfg}
    left_servo_cfg.update({
        "move_group_name": LaunchConfiguration("left_move_group_name"),
        "planning_frame": LaunchConfiguration("left_base_frame"),
        "ee_frame_name": LaunchConfiguration("left_ee_frame"),
        "robot_link_command_frame": LaunchConfiguration("left_ee_frame"),
        "cartesian_command_in_topic": LaunchConfiguration("left_twist_topic"),
        "status_topic": LaunchConfiguration("left_status_topic"),
        "command_out_topic": LaunchConfiguration("left_command_out_topic"),
    })
    right_servo_cfg = {**servo_sim_cfg}
    right_servo_cfg.update({
        "move_group_name": LaunchConfiguration("right_move_group_name"),
        "planning_frame": LaunchConfiguration("right_base_frame"),
        "ee_frame_name": LaunchConfiguration("right_ee_frame"),
        "robot_link_command_frame": LaunchConfiguration("right_ee_frame"),
        "cartesian_command_in_topic": LaunchConfiguration("right_twist_topic"),
        "status_topic": LaunchConfiguration("right_status_topic"),
        "command_out_topic": LaunchConfiguration("right_command_out_topic"),
    })

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
            single_moveit_config,
            servo_params_pt,
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'true' and '", control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(pose_tracking_node)

    # ---- 1b. servo_node_main (TwistStamped / JointJog → Servo) ----
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            single_moveit_config,
            servo_params,
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(servo_node)

    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name=LaunchConfiguration("left_servo_name"),
        output="screen",
        parameters=[
            dual_moveit_config_dict,
            {
                "moveit_servo": left_servo_cfg,
                "use_sim_time": True,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(left_servo_node)

    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name=LaunchConfiguration("right_servo_name"),
        output="screen",
        parameters=[
            dual_moveit_config_dict,
            {
                "moveit_servo": right_servo_cfg,
                "use_sim_time": True,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(right_servo_node)

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
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(start_servo)

    start_dual_servo = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "service", "call",
                     PythonExpression([
                         "'/' + '",
                         LaunchConfiguration("left_servo_name"),
                         "' + '/start_servo'",
                     ]),
                     "std_srvs/srv/Trigger", "{}"],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "service", "call",
                     PythonExpression([
                         "'/' + '",
                         LaunchConfiguration("right_servo_name"),
                         "' + '/start_servo'",
                     ]),
                     "std_srvs/srv/Trigger", "{}"],
                output="screen",
            ),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(start_dual_servo)

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
                      "orientation_tracking_mode",
                      "windup_limit", "filter_enabled", "filter_alpha",
                      "safe_zone_enabled",
                      "x_min", "x_max", "y_min", "y_max", "z_min", "z_max")},
            {"use_sim_time": True},
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(pose_tracking_node)

    def pose_tracking_params(target_topic, twist_topic, safe_zone_topic, base_frame, ee_frame):
        return [
            {k: v for k, v in pose_tracking_cfg.items()
             if k in ("x_proportional_gain", "y_proportional_gain",
                      "z_proportional_gain", "x_integral_gain",
                      "y_integral_gain", "z_integral_gain",
                      "x_derivative_gain", "y_derivative_gain",
                      "z_derivative_gain", "angular_proportional_gain",
                      "angular_integral_gain", "angular_derivative_gain",
                      "orientation_tracking_mode",
                      "windup_limit", "filter_enabled", "filter_alpha",
                      "safe_zone_enabled",
                      "x_min", "x_max", "y_min", "y_max", "z_min", "z_max")},
            {
                "target_topic": target_topic,
                "twist_topic": twist_topic,
                "safe_zone_topic": safe_zone_topic,
                "base_frame": base_frame,
                "ee_frame": ee_frame,
                "use_sim_time": True,
            },
        ]

    left_pose_tracking_node = Node(
        package="rm_dualarm",
        executable="pose_tracking_node.py",
        name="left_pose_tracking",
        output="screen",
        parameters=pose_tracking_params(
            LaunchConfiguration("left_target_topic"),
            LaunchConfiguration("left_twist_topic"),
            "/left/pose_tracking/safe_zone",
            LaunchConfiguration("left_base_frame"),
            LaunchConfiguration("left_ee_frame"),
        ),
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(left_pose_tracking_node)

    right_pose_tracking_node = Node(
        package="rm_dualarm",
        executable="pose_tracking_node.py",
        name="right_pose_tracking",
        output="screen",
        parameters=pose_tracking_params(
            LaunchConfiguration("right_target_topic"),
            LaunchConfiguration("right_twist_topic"),
            "/right/pose_tracking/safe_zone",
            LaunchConfiguration("right_base_frame"),
            LaunchConfiguration("right_ee_frame"),
        ),
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(right_pose_tracking_node)

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
            single_moveit_config,
            {"use_sim_time": True},
        ],
        condition=IfCondition(PythonExpression([
            "'", use_rviz, "' == 'true' and '", control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(rviz_node)

    dual_rviz_node = Node(
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
            dual_moveit_config_dict,
            {"use_sim_time": True},
        ],
        condition=IfCondition(PythonExpression([
            "'", use_rviz, "' == 'true' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(dual_rviz_node)

    # ---- 5. Teleop (mouse or VR, mutually exclusive) ----
    is_mouse = PythonExpression(["'", teleop_type, "' == 'mouse'"])
    is_vr = PythonExpression(["'", teleop_type, "' == 'vr'"])

    mouse_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "mouse_teleop.launch.py")
        ),
        condition=IfCondition(is_mouse),
        launch_arguments={
            "control_mode": control_mode,
            "publish_rate": LaunchConfiguration("mouse_publish_rate"),
            "fixed_x": LaunchConfiguration("fixed_x"),
            "y_min": LaunchConfiguration("y_min"),
            "y_max": LaunchConfiguration("y_max"),
            "left_target_topic": LaunchConfiguration("left_target_topic"),
            "right_target_topic": LaunchConfiguration("right_target_topic"),
            "left_active_topic": LaunchConfiguration("left_active_topic"),
            "right_active_topic": LaunchConfiguration("right_active_topic"),
            "left_base_frame": LaunchConfiguration("left_base_frame"),
            "right_base_frame": LaunchConfiguration("right_base_frame"),
            "left_ee_frame": LaunchConfiguration("left_ee_frame"),
            "right_ee_frame": LaunchConfiguration("right_ee_frame"),
        }.items(),
    )
    ld.add_action(mouse_teleop_launch)

    vr_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "vr_teleop.launch.py")
        ),
        condition=IfCondition(is_vr),
        launch_arguments={
            "control_mode": control_mode,
            "left_target_topic": LaunchConfiguration("left_target_topic"),
            "right_target_topic": LaunchConfiguration("right_target_topic"),
            "left_active_topic": LaunchConfiguration("left_active_topic"),
            "right_active_topic": LaunchConfiguration("right_active_topic"),
            "left_gripper_topic": LaunchConfiguration("left_gripper_topic"),
            "right_gripper_topic": LaunchConfiguration("right_gripper_topic"),
            "use_relay_receiver": LaunchConfiguration("use_relay_receiver"),
            "left_base_frame": LaunchConfiguration("left_base_frame"),
            "right_base_frame": LaunchConfiguration("right_base_frame"),
            "calibration_enabled": LaunchConfiguration("vr_calibration_enabled"),
            "calibration_file": LaunchConfiguration("vr_calibration_file"),
        }.items(),
    )
    ld.add_action(vr_teleop_launch)

    return ld
