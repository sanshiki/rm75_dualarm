#!/usr/bin/env python3
"""Merge left/right RM driver joint states into prefixed dual-arm joint_states."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DualJointStateMerger(Node):
    def __init__(self):
        super().__init__("dual_joint_state_merger")
        self.declare_parameter("left_joint_states_topic", "/left/joint_states")
        self.declare_parameter("right_joint_states_topic", "/right/joint_states")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("left_prefix", "left_")
        self.declare_parameter("right_prefix", "right_")

        self._left_prefix = self.get_parameter("left_prefix").value
        self._right_prefix = self.get_parameter("right_prefix").value
        self._left_msg = None
        self._right_msg = None

        self._pub = self.create_publisher(
            JointState, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            JointState,
            self.get_parameter("left_joint_states_topic").value,
            self._left_cb,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("right_joint_states_topic").value,
            self._right_cb,
            10,
        )
        self.get_logger().info("Merging /left and /right joint_states into /joint_states")

    def _left_cb(self, msg):
        self._left_msg = self._prefix_msg(msg, self._left_prefix)
        self._publish()

    def _right_cb(self, msg):
        self._right_msg = self._prefix_msg(msg, self._right_prefix)
        self._publish()

    def _prefix_msg(self, msg, prefix):
        out = JointState()
        out.header = msg.header
        out.name = [
            name if name.startswith(prefix) else f"{prefix}{name}"
            for name in msg.name
        ]
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)
        return out

    def _append_side(self, merged, side):
        merged.name.extend(side.name)
        merged.position.extend(side.position)
        if side.velocity:
            merged.velocity.extend(side.velocity)
        if side.effort:
            merged.effort.extend(side.effort)

    def _publish(self):
        if self._left_msg is None or self._right_msg is None:
            return
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = self._left_msg.header.frame_id or self._right_msg.header.frame_id
        self._append_side(merged, self._left_msg)
        self._append_side(merged, self._right_msg)
        self._pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = DualJointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
