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
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Point
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker
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


def _normalize_quat(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def _quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_error_vector(target, current):
    """Return shortest-path axis-angle error vector target * inverse(current)."""
    target = _normalize_quat(target)
    current = _normalize_quat(current)
    q_err = _normalize_quat(_quat_multiply(target, _quat_conjugate(current)))
    if q_err[3] < 0.0:
        q_err = tuple(-v for v in q_err)
    x, y, z, w = q_err
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half < 1e-9:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(sin_half, w)
    return angle * x / sin_half, angle * y / sin_half, angle * z / sin_half


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
        self.declare_parameter("orientation_tracking_mode", "full_quat")
        self.declare_parameter("windup_limit", 0.05)
        self.declare_parameter("ee_frame", "Link7")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("active_topic", "/teleop_active")
        self.declare_parameter("safe_zone_topic", "/pose_tracking/safe_zone")
        self.declare_parameter("max_linear", 0.2)
        self.declare_parameter("max_angular", 0.5)

        # ---- Low-pass filter (first-order: y[n] = α·x[n] + (1-α)·y[n-1]) ----
        self.declare_parameter("filter_enabled", False)
        self.declare_parameter("filter_alpha", 0.3)          # 0.0 = max smooth, 1.0 = bypass

        # ---- Safe zone (clamp target_pose before tracking) ----
        self.declare_parameter("safe_zone_enabled", True)
        self.declare_parameter("x_min", 0.15)
        self.declare_parameter("x_max", 0.60)
        self.declare_parameter("y_min", -0.40)
        self.declare_parameter("y_max", 0.40)
        self.declare_parameter("z_min", 0.10)
        self.declare_parameter("z_max", 0.80)

        # ---- Soft-start ramp (0 → 1 over ramp_time seconds) ----
        self.declare_parameter("soft_start_ramp_time", 2.0)    # 0.0 = disabled

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
        self._pid_ang_x = PID1D(
            self.get_parameter("angular_proportional_gain").value,
            self.get_parameter("angular_integral_gain").value,
            self.get_parameter("angular_derivative_gain").value, wu)
        self._pid_ang_y = PID1D(
            self.get_parameter("angular_proportional_gain").value,
            self.get_parameter("angular_integral_gain").value,
            self.get_parameter("angular_derivative_gain").value, wu)
        self._pid_ang_z = PID1D(
            self.get_parameter("angular_proportional_gain").value,
            self.get_parameter("angular_integral_gain").value,
            self.get_parameter("angular_derivative_gain").value, wu)

        self._ee_frame = self.get_parameter("ee_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._max_lin = self.get_parameter("max_linear").value
        self._max_ang = self.get_parameter("max_angular").value
        self._safe_on = self.get_parameter("safe_zone_enabled").value
        self._sx_min = self.get_parameter("x_min").value
        self._sx_max = self.get_parameter("x_max").value
        self._sy_min = self.get_parameter("y_min").value
        self._sy_max = self.get_parameter("y_max").value
        self._sz_min = self.get_parameter("z_min").value
        self._sz_max = self.get_parameter("z_max").value
        self._filter_on = self.get_parameter("filter_enabled").value
        self._filter_a = self.get_parameter("filter_alpha").value
        self._filter_a = max(0.0, min(1.0, self._filter_a))
        self._safe_zone_topic = self.get_parameter("safe_zone_topic").value
        self._ramp_time = self.get_parameter("soft_start_ramp_time").value
        if self._ramp_time < 0.001:
            self._ramp_time = 0.0   # disabled
        self._orientation_mode = self.get_parameter("orientation_tracking_mode").value
        if self._orientation_mode not in ("full_quat", "yaw_only"):
            self.get_logger().warn(
                f"Unknown orientation_tracking_mode={self._orientation_mode}; using full_quat")
            self._orientation_mode = "full_quat"

        # ---- Filter state (6-DOF: vx,vy,vz,wx,wy,wz) ----
        self._filt = [0.0] * 6
        self._filt_inited = False

        # ---- Soft-start state ----
        self._soft_start_scale = 0.0

        # ---- TF ----
        self._tf_buf = Buffer()
        self._tf_list = TransformListener(self._tf_buf, self)

        # ---- State ----
        self._target = None   # latest PoseStamped
        self._last_target_time = Time()
        self._teleop_active = False

        # ---- Pub / Sub ----
        self._twist_pub = self.create_publisher(
            TwistStamped, self.get_parameter("twist_topic").value, 10)
        self._viz_pub = self.create_publisher(
            Marker, self._safe_zone_topic, 1)
        self.create_subscription(
            PoseStamped, self.get_parameter("target_topic").value,
            self._target_cb, 10)
        self.create_subscription(
            Bool, self.get_parameter("active_topic").value,
            self._active_cb, 1)

        # ---- Control loop ----
        rate = max(self.get_parameter("publish_rate").value, 1.0)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        # Safe-zone marker is static; republish every 5 s for late joiners
        self._viz_timer = self.create_timer(5.0, self._publish_safe_zone)
        self._publish_safe_zone()   # also publish immediately
        self._last_tick = self.get_clock().now()

        self.get_logger().info(
            f"pose_tracking ready | ee={self._ee_frame} "
            f"| P=({self._pid_x.kp:.1f},{self._pid_y.kp:.1f},{self._pid_z.kp:.1f},{self._pid_ang.kp:.1f}) "
            f"| orientation={self._orientation_mode} "
            f"| filter={'on' if self._filter_on else 'off'} (α={self._filter_a:.2f}) "
            f"| safe={'on' if self._safe_on else 'off'} "
            f"X=[{self._sx_min:.2f},{self._sx_max:.2f}] "
            f"Y=[{self._sy_min:.2f},{self._sy_max:.2f}] "
            f"Z=[{self._sz_min:.2f},{self._sz_max:.2f}]"
            f"{' | soft_start=%.1fs' % self._ramp_time if self._ramp_time > 0.0 else ''}"
        )

    # ==================================================================
    def _publish_safe_zone(self):
        """Publish a wireframe cube Marker showing the safe-zone bounds."""
        cube = Marker()
        cube.header.stamp = self.get_clock().now().to_msg()
        cube.header.frame_id = self._base_frame
        cube.ns = "safe_zone"
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.lifetime = Duration(seconds=10).to_msg()

        x_range = self._sx_max - self._sx_min
        y_range = self._sy_max - self._sy_min
        z_range = self._sz_max - self._sz_min
        cube.pose.position.x = self._sx_min + x_range / 2.0
        cube.pose.position.y = self._sy_min + y_range / 2.0
        cube.pose.position.z = self._sz_min + z_range / 2.0
        cube.pose.orientation.w = 1.0
        cube.scale.x = x_range
        cube.scale.y = y_range
        cube.scale.z = z_range

        # Wireframe green, semi-transparent
        cube.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2)
        # Edges via LINE_LIST
        edge = Marker()
        edge.header = cube.header
        edge.ns = "safe_zone"
        edge.id = 1
        edge.type = Marker.LINE_LIST
        edge.action = Marker.ADD
        edge.lifetime = cube.lifetime
        edge.pose = cube.pose
        edge.scale.x = 0.005        # line width
        edge.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)

        # 12 edges of the cube
        rx, ry, rz = x_range / 2.0, y_range / 2.0, z_range / 2.0
        corners = [
            (-rx, -ry, -rz), ( rx, -ry, -rz), (-rx,  ry, -rz), ( rx,  ry, -rz),
            (-rx, -ry,  rz), ( rx, -ry,  rz), (-rx,  ry,  rz), ( rx,  ry,  rz),
        ]
        indices = [
            (0,1),(0,2),(1,3),(2,3),  # bottom
            (4,5),(4,6),(5,7),(6,7),  # top
            (0,4),(1,5),(2,6),(3,7),  # vertical
        ]
        pts = []
        for a, b in indices:
            pts.append(Point(x=corners[a][0], y=corners[a][1], z=corners[a][2]))
            pts.append(Point(x=corners[b][0], y=corners[b][1], z=corners[b][2]))
        edge.points = pts

        self._viz_pub.publish(cube)
        self._viz_pub.publish(edge)

    # ==================================================================
    def _target_cb(self, msg: PoseStamped):
        # Reset soft-start on first-ever target or when recovering from timeout
        if self._target is None or self._soft_start_scale < 1e-6:
            self._soft_start_scale = 0.0
        self._target = msg
        self._last_target_time = self.get_clock().now()

    def _active_cb(self, msg: Bool):
        self._teleop_active = msg.data
        if not self._teleop_active:
            self._reset_tracking_state(clear_target=True)

    # ==================================================================
    def _reset_tracking_state(self, clear_target=False):
        self._pid_x.reset()
        self._pid_y.reset()
        self._pid_z.reset()
        self._pid_ang.reset()
        self._pid_ang_x.reset()
        self._pid_ang_y.reset()
        self._pid_ang_z.reset()
        self._soft_start_scale = 0.0
        self._filt = [0.0] * 6
        self._filt_inited = False
        if clear_target:
            self._target = None

    def _get_ee_pose(self):
        """Return current EE (x, y, z, roll, pitch, yaw, quat) from TF."""
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
        return (x, y, z, roll, pitch, yaw, (r.x, r.y, r.z, r.w))

    # ==================================================================
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        if dt < 0.001 or dt > 0.5:
            dt = 0.02   # fallback

        # Check teleop active — stop immediately on deactivation
        if not self._teleop_active:
            self._reset_tracking_state(clear_target=True)
            return

        # Check timeout
        if self._target is None:
            return
        elapsed = (now - self._last_target_time).nanoseconds * 1e-9
        if elapsed > 0.3:   # no recent target → stay quiet
            self._reset_tracking_state(clear_target=True)
            # self.get_logger().warn(
                # f"target_pose timeout ({elapsed:.2f}s); reset soft-start and PID integrators")
            return

        ee = self._get_ee_pose()
        if ee is None:
            return
        ex, ey, ez, eroll, epitch, eyaw, current_q = ee

        # Position error — clamp target to safe zone first
        tx = self._target.pose.position.x
        ty = self._target.pose.position.y
        tz = self._target.pose.position.z
        if self._safe_on:
            tx = max(self._sx_min, min(self._sx_max, tx))
            ty = max(self._sy_min, min(self._sy_max, ty))
            tz = max(self._sz_min, min(self._sz_max, tz))
        err_x = tx - ex
        err_y = ty - ey
        err_z = tz - ez

        # Orientation error.
        tq = self._target.pose.orientation
        if self._orientation_mode == "yaw_only":
            _, _, tyaw = _quat_to_euler(tq.x, tq.y, tq.z, tq.w)
            ang_err = tyaw - eyaw
            ang_err = math.atan2(math.sin(ang_err), math.cos(ang_err))
            ang_err_x, ang_err_y, ang_err_z = 0.0, 0.0, ang_err
        else:
            ang_err_x, ang_err_y, ang_err_z = _quat_error_vector(
                (tq.x, tq.y, tq.z, tq.w), current_q)

        # ---- Soft-start ramp ----
        if self._ramp_time > 0.0:
            self._soft_start_scale = min(
                1.0, self._soft_start_scale + dt / self._ramp_time)
        else:
            self._soft_start_scale = 1.0

        # PID
        vx = self._pid_x.update(err_x, dt)
        vy = self._pid_y.update(err_y, dt)
        vz = self._pid_z.update(err_z, dt)
        if self._orientation_mode == "yaw_only":
            vw_x = 0.0
            vw_y = 0.0
            vw_z = self._pid_ang.update(ang_err_z, dt)
        else:
            vw_x = self._pid_ang_x.update(ang_err_x, dt)
            vw_y = self._pid_ang_y.update(ang_err_y, dt)
            vw_z = self._pid_ang_z.update(ang_err_z, dt)

        # Clamp
        vx = max(-self._max_lin, min(self._max_lin, vx))
        vy = max(-self._max_lin, min(self._max_lin, vy))
        vz = max(-self._max_lin, min(self._max_lin, vz))
        vw_x = max(-self._max_ang, min(self._max_ang, vw_x))
        vw_y = max(-self._max_ang, min(self._max_ang, vw_y))
        vw_z = max(-self._max_ang, min(self._max_ang, vw_z))

        # Apply soft-start scale
        s = self._soft_start_scale
        vx, vy, vz = vx * s, vy * s, vz * s
        vw_x, vw_y, vw_z = vw_x * s, vw_y * s, vw_z * s

        self._publish_twist(vx, vy, vz, vw_x, vw_y, vw_z, now)

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
