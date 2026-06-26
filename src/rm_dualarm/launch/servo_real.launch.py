# ================================================================
# MoveIt2 Servo — Standalone launch for real RM75 robot(s)
# ================================================================
# Starts ONLY servo-related nodes.  Start hardware bringup first:
#
#   ros2 launch rm_dualarm real_bringup.launch.py arm_ip:=192.168.1.18
#   ros2 launch rm_dualarm servo_real.launch.py control_mode:=single
#
#   ros2 launch rm_dualarm dual_real_bringup.launch.py \
#     left_arm_ip:=192.168.1.18 right_arm_ip:=192.168.1.19
#   ros2 launch rm_dualarm servo_real.launch.py control_mode:=dual
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

sys.path.append(os.path.dirname(__file__))
from dual_moveit_utils import dual_real_moveit_config  # noqa: E402


POSE_TRACKING_KEYS = (
    "x_proportional_gain", "y_proportional_gain", "z_proportional_gain",
    "x_integral_gain", "y_integral_gain", "z_integral_gain",
    "x_derivative_gain", "y_derivative_gain", "z_derivative_gain",
    "angular_proportional_gain", "angular_integral_gain", "angular_derivative_gain",
    "orientation_tracking_mode",
    "windup_limit", "filter_enabled", "filter_alpha",
    "safe_zone_enabled",
    "x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
)


def pose_tracking_base_params(pose_tracking_cfg):
    return {k: v for k, v in pose_tracking_cfg.items() if k in POSE_TRACKING_KEYS}


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory("rm_dualarm")

    for name, default in (
        ("use_pose_tracking", "false"),
        ("teleop_type", "mouse"),
        ("control_mode", "single"),
        ("fixed_x", "0.35"),
        ("mouse_publish_rate", "50.0"),
        ("y_min", "-0.35"),
        ("y_max", "0.35"),
        ("use_rviz", "true"),
        ("target_topic", "/target_pose"),
        ("twist_topic", "/servo_node/delta_twist_cmds"),
        ("status_topic", "/servo_node/status"),
        ("command_out_topic", "/servo_bridge/joint_trajectory_in"),
        ("active_topic", "/teleop_active"),
        ("base_frame", "base_link"),
        ("ee_frame", "Link7"),
        ("left_target_topic", "/left/target_pose"),
        ("right_target_topic", "/right/target_pose"),
        ("left_twist_topic", "/left_servo_node/delta_twist_cmds"),
        ("right_twist_topic", "/right_servo_node/delta_twist_cmds"),
        ("left_status_topic", "/left_servo_node/status"),
        ("right_status_topic", "/right_servo_node/status"),
        ("left_command_out_topic", "/left_servo_bridge/joint_trajectory_in"),
        ("right_command_out_topic", "/right_servo_bridge/joint_trajectory_in"),
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
        ("driver_topic", "/rm_driver/movej_canfd_cmd"),
        ("left_driver_topic", "/left/rm_driver/movej_canfd_cmd"),
        ("right_driver_topic", "/right/rm_driver/movej_canfd_cmd"),
        ("vr_calibration_enabled", "true"),
        ("vr_calibration_file", ""),
        ("base_x", ""),
        ("base_z", ""),
        ("base_spacing_y", ""),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    use_pose_tracking = LaunchConfiguration("use_pose_tracking")
    teleop_type = LaunchConfiguration("teleop_type")
    control_mode = LaunchConfiguration("control_mode")
    use_rviz = LaunchConfiguration("use_rviz")

    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .to_moveit_configs()
    )
    single_moveit_config = moveit_config.to_dict()
    dual_moveit_config_dict = dual_real_moveit_config(pkg_share)

    servo_real_cfg = yaml.safe_load(
        open(os.path.join(pkg_share, "config", "servo_75_real.yaml"))
    )
    pose_tracking_cfg = yaml.safe_load(
        open(os.path.join(pkg_share, "config", "pose_tracking_settings.yaml"))
    )

    single_servo_cfg = {**servo_real_cfg}
    single_servo_cfg.update({
        "planning_frame": LaunchConfiguration("base_frame"),
        "ee_frame_name": LaunchConfiguration("ee_frame"),
        "robot_link_command_frame": LaunchConfiguration("ee_frame"),
        "cartesian_command_in_topic": LaunchConfiguration("twist_topic"),
        "status_topic": LaunchConfiguration("status_topic"),
        "command_out_topic": LaunchConfiguration("command_out_topic"),
    })
    left_servo_cfg = {**servo_real_cfg}
    left_servo_cfg.update({
        "move_group_name": LaunchConfiguration("left_move_group_name"),
        "planning_frame": LaunchConfiguration("left_base_frame"),
        "ee_frame_name": LaunchConfiguration("left_ee_frame"),
        "robot_link_command_frame": LaunchConfiguration("left_ee_frame"),
        "cartesian_command_in_topic": LaunchConfiguration("left_twist_topic"),
        "status_topic": LaunchConfiguration("left_status_topic"),
        "command_out_topic": LaunchConfiguration("left_command_out_topic"),
    })
    right_servo_cfg = {**servo_real_cfg}
    right_servo_cfg.update({
        "move_group_name": LaunchConfiguration("right_move_group_name"),
        "planning_frame": LaunchConfiguration("right_base_frame"),
        "ee_frame_name": LaunchConfiguration("right_ee_frame"),
        "robot_link_command_frame": LaunchConfiguration("right_ee_frame"),
        "cartesian_command_in_topic": LaunchConfiguration("right_twist_topic"),
        "status_topic": LaunchConfiguration("right_status_topic"),
        "command_out_topic": LaunchConfiguration("right_command_out_topic"),
    })

    pose_tracking_single = {
        "moveit_servo": {**single_servo_cfg, **pose_tracking_cfg},
    }

    single_pose_tracking_node = Node(
        package="moveit_servo",
        executable="servo_pose_tracking_demo",
        name="servo_pose_tracking",
        output="screen",
        parameters=[single_moveit_config, pose_tracking_single],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'true' and '", control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(single_pose_tracking_node)

    single_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            single_moveit_config,
            {"moveit_servo": single_servo_cfg},
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(single_servo_node)

    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name=LaunchConfiguration("left_servo_name"),
        output="screen",
        parameters=[
            dual_moveit_config_dict,
            {"moveit_servo": left_servo_cfg},
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
            {"moveit_servo": right_servo_cfg},
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(right_servo_node)

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
                     PythonExpression(["'/' + '", LaunchConfiguration("left_servo_name"), "' + '/start_servo'"]),
                     "std_srvs/srv/Trigger", "{}"],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "service", "call",
                     PythonExpression(["'/' + '", LaunchConfiguration("right_servo_name"), "' + '/start_servo'"]),
                     "std_srvs/srv/Trigger", "{}"],
                output="screen",
            ),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(start_dual_servo)

    single_bridge = Node(
        package="rm_dualarm",
        executable="servo_bridge",
        name="servo_bridge",
        output="screen",
        parameters=[{
            "arm_dof": 7,
            "follow_mode": True,
            "publish_rate": 50.0,
            "command_timeout": 0.15,
            "halt_on_timeout": True,
            "servo_input_topic": LaunchConfiguration("command_out_topic"),
            "driver_topic": LaunchConfiguration("driver_topic"),
        }],
        condition=IfCondition(PythonExpression(["'", control_mode, "' == 'single'"])),
    )
    ld.add_action(single_bridge)

    left_bridge = Node(
        package="rm_dualarm",
        executable="servo_bridge",
        name="left_servo_bridge",
        output="screen",
        parameters=[{
            "arm_dof": 7,
            "follow_mode": True,
            "publish_rate": 50.0,
            "command_timeout": 0.15,
            "halt_on_timeout": True,
            "servo_input_topic": LaunchConfiguration("left_command_out_topic"),
            "driver_topic": LaunchConfiguration("left_driver_topic"),
        }],
        condition=IfCondition(PythonExpression(["'", control_mode, "' == 'dual'"])),
    )
    ld.add_action(left_bridge)

    right_bridge = Node(
        package="rm_dualarm",
        executable="servo_bridge",
        name="right_servo_bridge",
        output="screen",
        parameters=[{
            "arm_dof": 7,
            "follow_mode": True,
            "publish_rate": 50.0,
            "command_timeout": 0.15,
            "halt_on_timeout": True,
            "servo_input_topic": LaunchConfiguration("right_command_out_topic"),
            "driver_topic": LaunchConfiguration("right_driver_topic"),
        }],
        condition=IfCondition(PythonExpression(["'", control_mode, "' == 'dual'"])),
    )
    ld.add_action(right_bridge)

    base_pose_tracking_params = pose_tracking_base_params(pose_tracking_cfg)

    pose_tracking_node = Node(
        package="rm_dualarm",
        executable="pose_tracking_node.py",
        name="pose_tracking",
        output="screen",
        parameters=[
            base_pose_tracking_params,
            {
                "target_topic": LaunchConfiguration("target_topic"),
                "twist_topic": LaunchConfiguration("twist_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "use_sim_time": False,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", use_pose_tracking, "' == 'false' and '", control_mode, "' == 'single'"
        ])),
    )
    ld.add_action(pose_tracking_node)

    def dual_pose_tracking_params(target_topic, twist_topic, safe_zone_topic, base_frame, ee_frame):
        return [
            base_pose_tracking_params,
            {
                "target_topic": target_topic,
                "twist_topic": twist_topic,
                "safe_zone_topic": safe_zone_topic,
                "base_frame": base_frame,
                "ee_frame": ee_frame,
                "use_sim_time": False,
            },
        ]

    left_pose_tracking_node = Node(
        package="rm_dualarm",
        executable="pose_tracking_node.py",
        name="left_pose_tracking",
        output="screen",
        parameters=dual_pose_tracking_params(
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
        parameters=dual_pose_tracking_params(
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("rm_75_config"), "config", "moveit.rviz"),
        ],
        parameters=[single_moveit_config],
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
            os.path.join(get_package_share_directory("rm_dualarm"), "rviz", "moveit.rviz"),
        ],
        parameters=[dual_moveit_config_dict],
        condition=IfCondition(PythonExpression([
            "'", use_rviz, "' == 'true' and '", control_mode, "' == 'dual'"
        ])),
    )
    ld.add_action(dual_rviz_node)

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
