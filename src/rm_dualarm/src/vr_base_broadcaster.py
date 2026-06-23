#!/usr/bin/env python3
"""VR base frame broadcaster (ROS 2 port from noetic).

Listens to the headset pose (odom→headset TF) and publishes a vr_base
frame projected onto the ground plane (z=0) with optional headset yaw.
The vr_teleop node uses vr_base as its reference frame.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros import TransformException


def _euler_from_quat(x, y, z, w):
    """Return (roll, pitch, yaw) from quaternion [x, y, z, w]."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _quat_from_euler(roll, pitch, yaw):
    """Return [x, y, z, w] quaternion from euler angles."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def make_vr_base_transform(odom_frame, headset_tf, use_headset_yaw, stamp):
    """Build a vr_base TransformStamped from a headset TF.

    Projects headset XY to z=0 (ground plane).  Optionally keeps the
    headset yaw while zeroing roll/pitch.
    """
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = odom_frame
    t.child_frame_id = "vr_base"

    hx = headset_tf.transform.translation.x
    hy = headset_tf.transform.translation.y
    t.transform.translation.x = hx
    t.transform.translation.y = hy
    t.transform.translation.z = 0.0

    if use_headset_yaw:
        q = headset_tf.transform.rotation
        _, _, yaw = _euler_from_quat(q.x, q.y, q.z, q.w)
        q2 = _quat_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q2[0]
        t.transform.rotation.y = q2[1]
        t.transform.rotation.z = q2[2]
        t.transform.rotation.w = q2[3]
    else:
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

    return t


class VrBaseBroadcaster(Node):
    """Publishes the vr_base frame derived from the headset TF."""

    def __init__(self):
        super().__init__("vr_base_broadcaster")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("headset_frame", "headset")
        self.declare_parameter("use_headset_yaw", False)
        self.declare_parameter("rate", 30.0)

        self._odom = self.get_parameter("odom_frame").value
        self._headset = self.get_parameter("headset_frame").value
        self._use_yaw = self.get_parameter("use_headset_yaw").value
        rate = self.get_parameter("rate").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_br = TransformBroadcaster(self)

        period = 1.0 / max(rate, 1.0)
        self._timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f"vr_base_broadcaster: {self._odom}→{self._headset} → "
            f"{self._odom}→vr_base, use_yaw={self._use_yaw}"
        )

    def _timer_cb(self):
        try:
            now = Time()
            h_tf = self._tf_buffer.lookup_transform(
                self._odom, self._headset, now,
                Duration(seconds=0.5),
            )
        except TransformException:
            return

        vr_tf = make_vr_base_transform(
            self._odom, h_tf, self._use_yaw,
            self.get_clock().now().to_msg(),
        )
        self._tf_br.sendTransform(vr_tf)


def main(args=None):
    rclpy.init(args=args)
    node = VrBaseBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
