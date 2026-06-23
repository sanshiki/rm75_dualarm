#!/usr/bin/env python3
"""PoseStamped → TwistStamped converter with PID control.

Replaces servo_pose_tracking_demo for continuous servoing.
Subscribes to /target_pose, reads current EE pose from TF, runs
PID on position/orientation error, publishes TwistStamped to
/servo_node/delta_twist_cmds for servo_node_main.

PID gains are loaded from parameters (same as pose_tracking_settings.yaml).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from tf2_ros import Buffer, TransformListener, TransformException


def _quat_to_euler(x, y, z, w):
    """Quaternion → (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class PID1D:
    """Single-axis PID controller with anti-windup."""
    def __init__(self, kp, ki, kd, windup):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup = windup
        self._integral = 0.0
        self._prev_err = 0.0

    def update(self, err, dt):
        self._integral += err * dt
        self._integral = max(-self.windup, min(self.windup, self._integral))
        deriv = (err - self._prev_err) / dt if dt > 0.001 else 0.0
        self._prev_err = err
        return self.kp * err + self.ki * self._integral + self.kd * deriv

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0


class PoseTrackingNode(Node):
    def __init__(self):
        super().__init__("pose_tracking")

        # ---- PID gains (same names as pose_tracking_settings.yaml) ----
        self.declare_parameter("x_proportional_gain", 1.0)
        self.declare_parameter("y_proportional_gain", 1.0)
        self.declare_parameter("z_proportional_gain", 1.0)
        self.declare_parameter("x_integral_gain", 0.0)
        self.declare_parameter("y_integral_gain", 0.0)
        self.declare_parameter("z_integral_gain", 0.0)
        self.declare_parameter("x_derivative_gain", 0.0)
        self.declare_parameter("y_derivative_gain", 0.0)
        self.declare_parameter("z_derivative_gain", 0.0)
        self.declare_parameter("angular_proportional_gain", 0.5)
        self.declare_parameter("angular_integral_gain", 0.0)
        self.declare_parameter("angular_derivative_gain", 0.0)
        self.declare_parameter("windup_limit", 0.05)
        self.declare_parameter("ee_frame", "Link7")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("max_linear", 0.2)
        self.declare_parameter("max_angular", 0.5)

        # ---- Low-pass filter (first-order: y[n] = α·x[n] + (1-α)·y[n-1]) ----
        self.declare_parameter("filter_enabled", False)
        self.declare_parameter("filter_alpha", 0.3)          # 0.0 = max smooth, 1.0 = bypass

        # Read params
        wu = self.get_parameter("windup_limit").value
        self._pid_x = PID1D(
            self.get_parameter("x_proportional_gain").value,
            self.get_parameter("x_integral_gain").value,
            self.get_parameter("x_derivative_gain").value, wu)
        self._pid_y = PID1D(
            self.get_parameter("y_proportional_gain").value,
            self.get_parameter("y_integral_gain").value,
            self.get_parameter("y_derivative_gain").value, wu)
        self._pid_z = PID1D(
            self.get_parameter("z_proportional_gain").value,
            self.get_parameter("z_integral_gain").value,
            self.get_parameter("z_derivative_gain").value, wu)
        self._pid_ang = PID1D(
            self.get_parameter("angular_proportional_gain").value,
            self.get_parameter("angular_integral_gain").value,
            self.get_parameter("angular_derivative_gain").value, wu)

        self._ee_frame = self.get_parameter("ee_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._max_lin = self.get_parameter("max_linear").value
        self._max_ang = self.get_parameter("max_angular").value
        self._filter_on = self.get_parameter("filter_enabled").value
        self._filter_a = self.get_parameter("filter_alpha").value
        self._filter_a = max(0.0, min(1.0, self._filter_a))

        # ---- Filter state (6-DOF: vx,vy,vz,wx,wy,wz) ----
        self._filt = [0.0] * 6
        self._filt_inited = False

        # ---- TF ----
        self._tf_buf = Buffer()
        self._tf_list = TransformListener(self._tf_buf, self)

        # ---- State ----
        self._target = None   # latest PoseStamped
        self._last_target_time = Time()

        # ---- Pub / Sub ----
        self._twist_pub = self.create_publisher(
            TwistStamped, self.get_parameter("twist_topic").value, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter("target_topic").value,
            self._target_cb, 10)

        # ---- Control loop ----
        rate = max(self.get_parameter("publish_rate").value, 1.0)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self._last_tick = self.get_clock().now()

        self.get_logger().info(
            f"pose_tracking ready | ee={self._ee_frame} "
            f"| P=({self._pid_x.kp:.1f},{self._pid_y.kp:.1f},{self._pid_z.kp:.1f},{self._pid_ang.kp:.1f}) "
            f"| filter={'on' if self._filter_on else 'off'} (α={self._filter_a:.2f})"
        )

    # ==================================================================
    def _target_cb(self, msg: PoseStamped):
        self._target = msg
        self._last_target_time = self.get_clock().now()

    # ==================================================================
    def _get_ee_pose(self):
        """Return current EE (x, y, z, roll, pitch, yaw) from TF."""
        try:
            now = Time()
            t = self._tf_buf.lookup_transform(
                self._base_frame, self._ee_frame, now,
                Duration(seconds=0.5))
        except TransformException:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        r = t.transform.rotation
        roll, pitch, yaw = _quat_to_euler(r.x, r.y, r.z, r.w)
        return (x, y, z, roll, pitch, yaw)

    # ==================================================================
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        if dt < 0.001 or dt > 0.5:
            dt = 0.02   # fallback

        # Check timeout
        if self._target is None:
            return
        elapsed = (now - self._last_target_time).nanoseconds * 1e-9
        if elapsed > 0.3:   # no recent target → zero twist (stay)
            self._pid_x.reset()
            self._pid_y.reset()
            self._pid_z.reset()
            self._pid_ang.reset()
            self._publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, now)
            return

        ee = self._get_ee_pose()
        if ee is None:
            return
        ex, ey, ez, eroll, epitch, eyaw = ee

        # Position error
        tx = self._target.pose.position.x
        ty = self._target.pose.position.y
        tz = self._target.pose.position.z
        err_x = tx - ex
        err_y = ty - ey
        err_z = tz - ez

        # Orientation error (yaw only — roll/pitch held at 0 for safety)
        tq = self._target.pose.orientation
        _, _, tyaw = _quat_to_euler(tq.x, tq.y, tq.z, tq.w)
        ang_err = tyaw - eyaw
        # Wrap to [-pi, pi]
        ang_err = math.atan2(math.sin(ang_err), math.cos(ang_err))

        # PID
        vx = self._pid_x.update(err_x, dt)
        vy = self._pid_y.update(err_y, dt)
        vz = self._pid_z.update(err_z, dt)
        vw = self._pid_ang.update(ang_err, dt)

        # Clamp
        vx = max(-self._max_lin, min(self._max_lin, vx))
        vy = max(-self._max_lin, min(self._max_lin, vy))
        vz = max(-self._max_lin, min(self._max_lin, vz))
        vw = max(-self._max_ang, min(self._max_ang, vw))

        self._publish_twist(vx, vy, vz, 0.0, 0.0, vw, now)

    def _apply_filter(self, vx, vy, vz, wx, wy, wz):
        """First-order low-pass: y[n] = α·x[n] + (1-α)·y[n-1]."""
        if not self._filter_on:
            return vx, vy, vz, wx, wy, wz
        raw = [vx, vy, vz, wx, wy, wz]
        if not self._filt_inited:
            self._filt = raw[:]
            self._filt_inited = True
            return raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]
        a = self._filter_a
        self._filt = [a * r + (1.0 - a) * f
                      for r, f in zip(raw, self._filt)]
        return tuple(self._filt)

    def _publish_twist(self, vx, vy, vz, wx, wy, wz, stamp):
        vx, vy, vz, wx, wy, wz = self._apply_filter(vx, vy, vz, wx, wy, wz)
        msg = TwistStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear = Vector3(x=vx, y=vy, z=vz)
        msg.twist.angular = Vector3(x=wx, y=wy, z=wz)
        self._twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
