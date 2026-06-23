#!/usr/bin/env python3
"""Gated relay for JointTrajectory — activates when teleop signals ready.

Subscribes to the servo's raw JointTrajectory output and to
/teleop_active (std_msgs/Bool).  Forwarding is gated until the teleop
node (mouse or VR) publishes True on /teleop_active.

The servo's own /target_pose self-publish does NOT trigger this gate.
Only the dedicated /teleop_active topic from the teleop nodes does.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool


class TrajectoryRelay(Node):
    def __init__(self):
        super().__init__("trajectory_relay")

        self.declare_parameter("input_topic",
                               "/rm_group_controller/joint_trajectory_raw")
        self.declare_parameter("output_topic",
                               "/rm_group_controller/joint_trajectory")
        self.declare_parameter("settle_seconds", 2.0)
        self.declare_parameter("max_wait", 30.0)

        settle = self.get_parameter("settle_seconds").value
        max_wait = self.get_parameter("max_wait").value

        # Two-phase gate: teleop_active triggers settle timer;
        # actual forwarding only starts after settle timer fires.
        self._gated = True         # drop all messages while True
        self._settle_pending = False

        self._pub = self.create_publisher(
            JointTrajectory, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            JointTrajectory, self.get_parameter("input_topic").value,
            self._traj_cb, 10)
        self.create_subscription(
            Bool, "/teleop_active", self._gate_cb, 1)

        # Safety net: force-open after max_wait regardless
        self.create_timer(max_wait, self._force_activate)

        self.get_logger().info(
            f"Trajectory relay GATED on /teleop_active "
            f"+ {settle:.1f}s settle (or {max_wait:.0f}s timeout)"
        )

    def _gate_cb(self, msg: Bool):
        if self._gated and msg.data and not self._settle_pending:
            settle = self.get_parameter("settle_seconds").value
            self._settle_pending = True
            self.get_logger().info(
                f"Teleop active — settling {settle:.1f}s before open ..."
            )
            self.create_timer(settle, self._open_gate)

    def _open_gate(self):
        self._gated = False
        self.get_logger().info("Trajectory relay OPEN — forwarding")

    def _force_activate(self):
        if self._gated:
            self._gated = False
            self.get_logger().warn(
                "Trajectory relay force-opened by timeout"
            )

    def _traj_cb(self, msg: JointTrajectory):
        if not self._gated:
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
