import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topics = [
        # --- teleop target (sim & real) ---
        "/joint_states",
        "/target_pose",
        "/tf",
        "/tf_static",
        # --- teleop active ---
        "/teleop_active",
        # --- servo twist & status ---
        "/servo_node/delta_twist_cmds",
        "/servo_node/status",
        # --- sim: servo → MoveIt controller ---
        "/rm_group_controller/joint_trajectory",
        # --- real: servo → servo_bridge ---
        "/servo_bridge/joint_trajectory_in",
        # --- real: servo_bridge → rm_driver ---
        "/rm_driver/movej_canfd_cmd",
        # --- gripper ---
        "/rm_driver/set_gripper_position_cmd",
        # --- camera ---
        "/camera/image_raw",
        "/wrist_camera/color/image_raw",
    ]
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    default_output = f"bags/single_teleop_{timestamp}"
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value=default_output),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", LaunchConfiguration("output")] + topics,
            output="screen",
        ),
    ])
