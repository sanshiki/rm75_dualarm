from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topics = [
        "/joint_states",
        "/left/target_pose",
        "/right/target_pose",
        "/left_servo_node/delta_twist_cmds",
        "/right_servo_node/delta_twist_cmds",
        "/left_rm_group_controller/joint_trajectory",
        "/right_rm_group_controller/joint_trajectory",
        "/tf",
        "/tf_static",
    ]
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="bags/dual_teleop"),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", LaunchConfiguration("output")] + topics,
            output="screen",
        ),
    ])
