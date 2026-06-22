#!/usr/bin/env python3
"""Move the RM75 arm to a fixed non-singular joint pose via move_group.

Call this BEFORE launching servo_sim to initialise the arm away from
the singular zero position::

    ros2 run rm_dualarm pose_init.py
"""

import sys
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


# Fixed non-singular pose — joint order as move_group expects
# (joint1..joint7 from the rm_group planning group).
_INIT_JOINTS = {
    "joint1": -0.022524496509642944,
    "joint2":  0.263926554708664440,
    "joint3": -0.011319336648978329,
    "joint4":  1.293130423737138400,
    "joint5": -0.008347275735425264,
    "joint6":  1.323508237049473700,
    "joint7":  0.217822762411817200,
}

_PLANNING_GROUP = "rm_group"
_EXPECTED_JOINTS = ["joint1", "joint2", "joint3", "joint4",
                    "joint5", "joint6", "joint7"]


class PoseInitNode(Node):
    """Sends a MoveGroup action to plan + execute."""

    def __init__(self):
        super().__init__("pose_init")

        self.declare_parameter("delay_seconds", 15.0)
        delay = self.get_parameter("delay_seconds").value

        self.get_logger().info(
            f"Waiting {delay:.0f} s for Gazebo + move_group to stabilise ..."
        )
        self._init_timer = self.create_timer(delay, self._connect_and_send)

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
        self.get_logger().info("move_group found, sending goal ...")
        self.send_goal()

    def send_goal(self):
        goal = MoveGroup.Goal()

        # ---- Request ----
        goal.request = MotionPlanRequest()
        goal.request.group_name = _PLANNING_GROUP
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        # Joint constraints — one per joint
        constraints = Constraints()
        for jn in _EXPECTED_JOINTS:
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = _INIT_JOINTS[jn]
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
            "Goal joints: " +
            ", ".join(f"{jn}={_INIT_JOINTS[jn]:.3f}"
                      for jn in _EXPECTED_JOINTS[:4]) +
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
            self.get_logger().info("Pose init SUCCESS — arm in position")
        else:
            self.get_logger().error(
                f"Pose init FAILED: error_code={result.error_code.val}"
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
