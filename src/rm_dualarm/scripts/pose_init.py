#!/usr/bin/env python3
"""Move RM75 arm(s) to a fixed non-singular joint pose via move_group.

Call this BEFORE launching servo_sim to initialise the arm away from
the singular zero position::

    ros2 run rm_dualarm pose_init.py

For dual-arm simulation, pass ``control_mode:=dual``. The same standby
joint pose is sent to ``left_rm_group`` and ``right_rm_group`` with the
matching joint-name prefixes.
"""

import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
)
from builtin_interfaces.msg import Duration


_PLANNING_GROUP = "rm_group"
_EXPECTED_JOINTS = ["joint1", "joint2", "joint3", "joint4",
                    "joint5", "joint6", "joint7"]
_DUAL_TARGETS = [
    ("left_rm_group", "left_"),
    ("right_rm_group", "right_"),
]


class PoseInitNode(Node):
    """Sends a MoveGroup action to plan + execute."""

    def __init__(self):
        super().__init__("pose_init")

        self.declare_parameter("delay_seconds", 15.0)
        self.declare_parameter("control_mode", "single")
        self.declare_parameter("standby_pose_file", "")
        delay = self.get_parameter("delay_seconds").value
        self._control_mode = self.get_parameter("control_mode").value
        self._base_init_joints = self._load_standby_joints()
        self._targets = self._make_targets()
        self._target_index = 0
        self._action = None

        self.get_logger().info(
            f"Waiting {delay:.0f} s for Gazebo + move_group to stabilise "
            f"({self._control_mode}, {len(self._targets)} target(s)) ..."
        )
        self._init_timer = self.create_timer(delay, self._connect_and_send)

    def _default_standby_pose_file(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(
                get_package_share_directory("rm_dualarm"),
                "config",
                "standby_pose.yaml",
            )
        except Exception:
            return os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                "config",
                "standby_pose.yaml",
            )

    def _load_standby_joints(self):
        path = self.get_parameter("standby_pose_file").value
        if not path:
            path = self._default_standby_pose_file()
        if not os.path.exists(path):
            self.get_logger().fatal(f"Standby pose file not found: {path}")
            rclpy.shutdown()
            raise RuntimeError(f"standby pose file not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        joints = data.get("joints", data)
        missing = [name for name in _EXPECTED_JOINTS if name not in joints]
        if missing:
            raise RuntimeError(
                f"Standby pose file {path} missing joints: {', '.join(missing)}"
            )
        loaded = {name: float(joints[name]) for name in _EXPECTED_JOINTS}
        self.get_logger().info(f"Loaded standby pose: {path}")
        return loaded

    def _make_targets(self):
        if self._control_mode == "dual":
            return [
                (group, {
                    f"{prefix}{joint}": position
                    for joint, position in self._base_init_joints.items()
                })
                for group, prefix in _DUAL_TARGETS
            ]
        return [(_PLANNING_GROUP, dict(self._base_init_joints))]

    def _connect_and_send(self):
        self._init_timer.cancel()   # one-shot (oneshot= not in Humble)
        self._action = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("Waiting for /move_action server ...")
        if not self._action.wait_for_server(timeout_sec=30.0):
            self.get_logger().fatal(
                "/move_action not available — is move_group running?"
            )
            rclpy.shutdown()
            return
        self.get_logger().info("move_group found, sending standby goal(s) ...")
        self.send_next_goal()

    def send_next_goal(self):
        if self._target_index >= len(self._targets):
            self.get_logger().info("Pose init SUCCESS — all arm(s) in position")
            rclpy.shutdown()
            return

        group_name, joint_positions = self._targets[self._target_index]
        goal = MoveGroup.Goal()

        # ---- Request ----
        goal.request = MotionPlanRequest()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        # Joint constraints — one per joint
        constraints = Constraints()
        for jn in joint_positions:
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = joint_positions[jn]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        # ---- Planning options ----
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False    # plan + execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.replan_delay = 1.0

        self.get_logger().info(
            f"[{group_name}] goal joints: " +
            ", ".join(f"{jn}={joint_positions[jn]:.3f}"
                      for jn in list(joint_positions)[:4]) +
            " ..."
        )

        future = self._action.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by move_group")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted, waiting for result ...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            group_name, _ = self._targets[self._target_index]
            self.get_logger().info(f"[{group_name}] standby pose reached")
            self._target_index += 1
            self.send_next_goal()
        else:
            group_name, _ = self._targets[self._target_index]
            self.get_logger().error(
                f"[{group_name}] pose init FAILED: error_code={result.error_code.val}"
            )
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PoseInitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
