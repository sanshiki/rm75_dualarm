import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topics = [
        # --- teleop targets (sim & real) ---
        "/joint_states",
        "/left/target_pose",
        "/right/target_pose",
        "/tf",
        "/tf_static",
        # --- servo twist commands (sim & real) ---
        "/left_servo_node/delta_twist_cmds",
        "/right_servo_node/delta_twist_cmds",
        # --- sim: servo → MoveIt controller ---
        "/left_rm_group_controller/joint_trajectory",
        "/right_rm_group_controller/joint_trajectory",
        # --- real: servo → servo_bridge ---
        "/left_servo_bridge/joint_trajectory_in",
        "/right_servo_bridge/joint_trajectory_in",
        # --- real: servo_bridge → rm_driver ---
        "/left/rm_driver/movej_canfd_cmd",
        "/right/rm_driver/movej_canfd_cmd",
    ]
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    default_output = f"bags/dual_teleop_{timestamp}"
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value=default_output),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", LaunchConfiguration("output")] + topics,
            output="screen",
        ),
    ])
