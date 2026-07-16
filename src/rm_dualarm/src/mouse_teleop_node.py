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
from std_msgs.msg import Bool
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
        self.declare_parameter("control_mode", "single")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("left_target_topic", "/left/target_pose")
        self.declare_parameter("right_target_topic", "/right/target_pose")
        self.declare_parameter("active_topic", "/teleop_active")
        self.declare_parameter("left_active_topic", "/left/teleop_active")
        self.declare_parameter("right_active_topic", "/right/teleop_active")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_base_frame", "left_base_link")
        self.declare_parameter("right_base_frame", "right_base_link")
        self.declare_parameter("ee_frame", "Link7")
        self.declare_parameter("left_ee_frame", "left_Link7")
        self.declare_parameter("right_ee_frame", "right_Link7")

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
        self.declare_parameter("dual_y_offset", 0.25)

        publish_rate = self.get_parameter("publish_rate").value
        self._control_mode = self.get_parameter("control_mode").value
        self._target_topic = self.get_parameter("target_topic").value
        self._left_target_topic = self.get_parameter("left_target_topic").value
        self._right_target_topic = self.get_parameter("right_target_topic").value
        self._active_topic = self.get_parameter("active_topic").value
        self._left_active_topic = self.get_parameter("left_active_topic").value
        self._right_active_topic = self.get_parameter("right_active_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._left_base_frame = self.get_parameter("left_base_frame").value
        self._right_base_frame = self.get_parameter("right_base_frame").value
        self._ee_frame = self.get_parameter("ee_frame").value
        self._left_ee_frame = self.get_parameter("left_ee_frame").value
        self._right_ee_frame = self.get_parameter("right_ee_frame").value
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
        self._dual_y_offset = self.get_parameter("dual_y_offset").value

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
        self._static_quat_attempted = False
        self._left_fixed_q = None
        self._right_fixed_q = None

        # ---- Publisher ----
        self._pose_pub = self.create_publisher(PoseStamped, self._target_topic, 10)
        self._left_pose_pub = self.create_publisher(PoseStamped, self._left_target_topic, 10)
        self._right_pose_pub = self.create_publisher(PoseStamped, self._right_target_topic, 10)
        self._active_pub = self.create_publisher(Bool, self._active_topic, 1)
        self._left_active_pub = self.create_publisher(Bool, self._left_active_topic, 1)
        self._right_active_pub = self.create_publisher(Bool, self._right_active_topic, 1)

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
            f"mode={self._control_mode}  |  "
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
                self.get_logger().info("Servo ACTIVE - robot is following mouse")
            else:
                self._publish_active(False)
                self.get_logger().info("Servo IDLE - holding position (timeout)")

    def _publish_active(self, active):
        msg = Bool(data=active)
        if self._control_mode == "dual":
            self._left_active_pub.publish(msg)
            self._right_active_pub.publish(msg)
        else:
            self._active_pub.publish(msg)

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
    def _lookup_eef_quat(self, base_frame=None, ee_frame=None):
        """Return the current end-effector quaternion from TF."""
        base_frame = base_frame or self._base_frame
        ee_frame = ee_frame or self._ee_frame
        try:
            now = Time()
            t = self._tf_buffer.lookup_transform(
                base_frame, ee_frame, now, Duration(seconds=0.05),
            )
            r = t.transform.rotation
            return r.x, r.y, r.z, r.w
        except TransformException as e:
            self.get_logger().warn(f"TF lookup {base_frame}->{ee_frame} failed: {e}")
            return None

    # ==================================================================
    def _make_pose(self, frame_id, x, y, z, quat=None):
        qx, qy, qz, qw = quat or (
            self._fixed_qx, self._fixed_qy, self._fixed_qz, self._fixed_qw)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return pose

    def _capture_static_quat_once(self):
        if not self._static_quat or self._static_quat_attempted:
            return
        self._static_quat_attempted = True

        if self._control_mode == "dual":
            left_q = self._lookup_eef_quat(self._left_base_frame, self._left_ee_frame)
            right_q = self._lookup_eef_quat(self._right_base_frame, self._right_ee_frame)
            if left_q is not None:
                self._left_fixed_q = left_q
            if right_q is not None:
                self._right_fixed_q = right_q
            if left_q is not None or right_q is not None:
                self.get_logger().info("Static quat captured for dual mouse teleop")
            else:
                self.get_logger().warn(
                    "Static quat requested but dual EE TF was unavailable; "
                    "using configured fixed quaternion"
                )
            self._static_quat_recorded = True
            return

        q = self._lookup_eef_quat()
        if q is not None:
            self._fixed_qx, self._fixed_qy, self._fixed_qz, self._fixed_qw = q
            self.get_logger().info(
                f"Static quat captured from TF: "
                f"({self._fixed_qx:.4f}, {self._fixed_qy:.4f}, "
                f"{self._fixed_qz:.4f}, {self._fixed_qw:.4f})"
            )
        else:
            self.get_logger().warn(
                "Static quat requested but EE TF was unavailable; "
                "using configured fixed quaternion"
            )
        self._static_quat_recorded = True

    def _timer_callback(self):
        if not self._left_held:
            return   # quiet when released — Servo times out and halts

        # On first activation, optionally snap orientation from EE pose.
        # Never retry inside the high-rate timer; missing TF would throttle teleop.
        self._capture_static_quat_once()

        rx, ry, rz = self._screen_to_robot()

        if self._control_mode == "dual":
            left_pose = self._make_pose(
                self._left_base_frame, rx, ry + self._dual_y_offset, rz,
                self._left_fixed_q)
            right_pose = self._make_pose(
                self._right_base_frame, rx, ry - self._dual_y_offset, rz,
                self._right_fixed_q)
            self._left_pose_pub.publish(left_pose)
            self._right_pose_pub.publish(right_pose)
            self._publish_active(True)
        else:
            self._pose_pub.publish(self._make_pose(self._base_frame, rx, ry, rz))
            self._publish_active(True)


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
