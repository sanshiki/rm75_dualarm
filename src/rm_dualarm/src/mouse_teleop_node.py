#!/usr/bin/env python3
"""Mouse-to-Cartesian teleoperation — absolute screen-to-plane mapping.

Maps the absolute 2-D mouse position on the screen directly to a 3-D
position on a fixed plane in front of the robot.  Orientation is fixed
to a constant quaternion to avoid rotational singularities.

Hold LEFT button  →  publish to /target_pose (Servo follows)
Release           →  stop publishing (Servo halts on timeout)
"""

import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_ros import Buffer, TransformListener, TransformException

try:
    from pynput.mouse import Listener as MouseListener, Button as MouseButton
    PYNPUT_OK = True
except ImportError:
    PYNPUT_OK = False


class MouseTeleopNode(Node):
    """Absolute mouse → Cartesian plane mapping for RM75 arm teleop."""

    def __init__(self):
        super().__init__("mouse_teleop")

        if not PYNPUT_OK:
            self.get_logger().fatal("pynput missing — pip install pynput")
            raise RuntimeError("pynput missing")

        # ---- Parameters ----
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("base_frame", "base_link")

        # Fixed X distance (forward from base) [m]
        self.declare_parameter("fixed_x", 0.35)
        # Y range the screen width maps to [m]
        self.declare_parameter("y_min", -0.35)
        self.declare_parameter("y_max", 0.35)
        # Z range the screen height maps to [m]
        self.declare_parameter("z_min", 0.15)
        self.declare_parameter("z_max", 0.70)
        # Fixed orientation [x, y, z, w]
        self.declare_parameter("fixed_qx", 0.0)
        self.declare_parameter("fixed_qy", 0.0)
        self.declare_parameter("fixed_qz", 0.0)
        self.declare_parameter("fixed_qw", 1.0)
        self.declare_parameter("static_quat", False)

        publish_rate = self.get_parameter("publish_rate").value
        self._target_topic = self.get_parameter("target_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._fixed_x = self.get_parameter("fixed_x").value
        self._y_min = self.get_parameter("y_min").value
        self._y_max = self.get_parameter("y_max").value
        self._z_min = self.get_parameter("z_min").value
        self._z_max = self.get_parameter("z_max").value
        self._static_quat = self.get_parameter("static_quat").value
        self._fixed_qx = self.get_parameter("fixed_qx").value
        self._fixed_qy = self.get_parameter("fixed_qy").value
        self._fixed_qz = self.get_parameter("fixed_qz").value
        self._fixed_qw = self.get_parameter("fixed_qw").value

        # ---- Get screen dimensions via tkinter (stdlib) ----
        root = tk.Tk()
        self._screen_w = root.winfo_screenwidth()
        self._screen_h = root.winfo_screenheight()
        root.destroy()
        # pynput reports coordinates in the same pixel space as tkinter

        # ---- State ----
        self._left_held = False
        self._mouse_x = self._screen_w // 2   # start at screen centre
        self._mouse_y = self._screen_h // 2
        self._static_quat_recorded = False

        # ---- Publisher ----
        self._pose_pub = self.create_publisher(PoseStamped, self._target_topic, 10)

        # ---- TF (for reading current EE orientation) ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)


        # ---- Mouse listener ----
        self._listener = MouseListener(
            on_move=self._on_move,
            on_click=self._on_click,
        )
        self._listener.daemon = True
        self._listener.start()

        # ---- Timer (publishes continuously when button held) ----
        period = 1.0 / max(publish_rate, 1.0)
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f"Mouse teleop ready  |  screen={self._screen_w}x{self._screen_h}  |  "
            f"fixed_x={self._fixed_x:.2f}  Y=[{self._y_min:.2f}, {self._y_max:.2f}]  "
            f"Z=[{self._z_min:.2f}, {self._z_max:.2f}]  |  "
            f"Hold LEFT to activate, release to stop"
        )

    # ==================================================================
    def _on_move(self, x, y):
        self._mouse_x = x
        self._mouse_y = y

    def _on_click(self, x, y, button, pressed):
        if button == MouseButton.left:
            self._left_held = pressed
            if pressed:
                self.get_logger().info("Servo ACTIVE — robot is following mouse")
            else:
                self.get_logger().info("Servo IDLE — holding position (timeout)")

    # ==================================================================
    def _screen_to_robot(self):
        """Map absolute screen pixel → robot YZ on the fixed-X plane."""
        # Normalise 0..1
        nx = self._mouse_x / float(self._screen_w)
        ny = self._mouse_y / float(self._screen_h)

        # Map to robot workspace: screen X→Y, screen Y→Z (inverted)
        ry = self._y_min + nx * (self._y_max - self._y_min)
        rz = self._z_max - ny * (self._z_max - self._z_min)  # screen top = high Z

        return self._fixed_x, ry, rz

    # ==================================================================
    def _lookup_eef_quat(self):
        """Return the current end-effector quaternion from TF."""
        try:
            now = Time()
            t = self._tf_buffer.lookup_transform(
                self._base_frame, "Link7", now, Duration(seconds=0.5),
            )
            r = t.transform.rotation
            return r.x, r.y, r.z, r.w
        except TransformException as e:
            self.get_logger().warn(f"TF lookup Link7 failed: {e}")
            return None

    # ==================================================================
    def _timer_callback(self):
        if not self._left_held:
            return   # quiet when released — Servo times out and halts

        # On first activation, snap orientation from the real EE pose
        if self._static_quat and not self._static_quat_recorded:
            q = self._lookup_eef_quat()
            if q is not None:
                self._fixed_qx, self._fixed_qy, self._fixed_qz, self._fixed_qw = q
                self._static_quat_recorded = True
                self.get_logger().info(
                    f"Static quat captured from TF: "
                    f"({self._fixed_qx:.4f}, {self._fixed_qy:.4f}, "
                    f"{self._fixed_qz:.4f}, {self._fixed_qw:.4f})"
                )

        rx, ry, rz = self._screen_to_robot()

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._base_frame
        pose.pose.position = Point(x=rx, y=ry, z=rz)
        pose.pose.orientation = Quaternion(
            x=self._fixed_qx, y=self._fixed_qy,
            z=self._fixed_qz, w=self._fixed_qw,
        )

        self._pose_pub.publish(pose)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MouseTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down ...")
        node._listener.stop()
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
