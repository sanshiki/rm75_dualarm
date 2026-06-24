#!/usr/bin/env python3
"""VR hand-tracking teleoperation node (ROS 2 port from noetic).

Subscribes to /quest/joystick (Joy) and TF from the VR headset to map
the user's hand pose directly to the robot end-effector target.

Two control modes:
  normal       — absolute VR hand pose → robot target
  incremental  — VR hand delta → robot target (accumulated from EE pose)

Activation:  triple-press A
Block:       hold RB
Mode toggle: press B

Output: PoseStamped on /target_pose (consumed by servo_pose_tracking_demo).
"""

from queue import Queue
import os
import yaml

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformListener, TransformException


# Remap VR coordinate frame to robot frame.
# VR: X=right, Y=up, Z=backward  →  Robot: X=forward, Y=left, Z=up
ORI_MAPPING = np.array([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0],
])


class ButtonState:
    """Track button edge transitions (press / release)."""

    def __init__(self, name, on_press=None, on_release=None):
        self.name = name
        self.state = False
        self.last_state = False
        self.on_press = on_press
        self.on_release = on_release

    def update(self, state):
        self.last_state = self.state
        self.state = state
        if self.state and not self.last_state:
            if self.on_press:
                self.on_press()
        elif not self.state and self.last_state:
            if self.on_release:
                self.on_release()


class VRTrackerNode(Node):
    """ROS 2 node that maps VR hand poses to robot target poses."""

    def __init__(self):
        super().__init__("vr_teleop")

        # ---- Parameters ----
        self.declare_parameter("control_mode", "single")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("left_target_topic", "/left/target_pose")
        self.declare_parameter("right_target_topic", "/right/target_pose")
        self.declare_parameter("active_topic", "/teleop_active")
        self.declare_parameter("left_active_topic", "/left/teleop_active")
        self.declare_parameter("right_active_topic", "/right/teleop_active")
        self.declare_parameter("left_gripper_topic", "/left/gripper_cmd")
        self.declare_parameter("right_gripper_topic", "/right/gripper_cmd")
        self.declare_parameter("joy_topic", "/quest/joystick")
        self.declare_parameter("vr_base_frame", "vr_base")
        self.declare_parameter("vr_origin_frame", "vr_origin")
        self.declare_parameter("vr_hand_frame", "hand_right")
        self.declare_parameter("left_vr_hand_frame", "hand_left")
        self.declare_parameter("right_vr_hand_frame", "hand_right")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_base_frame", "left_base_link")
        self.declare_parameter("right_base_frame", "right_base_link")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("p_sensitivity", 4.0)
        self.declare_parameter("q_sensitivity", 4.0)
        self.declare_parameter("user_height", 1.75)
        self.declare_parameter("dual_y_offset", 0.0)
        self.declare_parameter("calibration_enabled", True)
        self.declare_parameter("calibration_file", "")

        self._control_mode = self.get_parameter("control_mode").value
        self._target_topic = self.get_parameter("target_topic").value
        self._left_target_topic = self.get_parameter("left_target_topic").value
        self._right_target_topic = self.get_parameter("right_target_topic").value
        self._active_topic = self.get_parameter("active_topic").value
        self._left_active_topic = self.get_parameter("left_active_topic").value
        self._right_active_topic = self.get_parameter("right_active_topic").value
        self._left_gripper_topic = self.get_parameter("left_gripper_topic").value
        self._right_gripper_topic = self.get_parameter("right_gripper_topic").value
        self._vr_base = self.get_parameter("vr_base_frame").value
        self._vr_origin = self.get_parameter("vr_origin_frame").value
        self._hand_frame = self.get_parameter("vr_hand_frame").value
        self._left_hand_frame = self.get_parameter("left_vr_hand_frame").value
        self._right_hand_frame = self.get_parameter("right_vr_hand_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._left_base_frame = self.get_parameter("left_base_frame").value
        self._right_base_frame = self.get_parameter("right_base_frame").value
        self._pub_rate = self.get_parameter("publish_rate").value
        self._p_sensitivity = self.get_parameter("p_sensitivity").value
        self._q_sensitivity = self.get_parameter("q_sensitivity").value
        self._user_height = self.get_parameter("user_height").value
        self._dual_y_offset = self.get_parameter("dual_y_offset").value
        self._calibration_enabled = self.get_parameter("calibration_enabled").value
        self._calibration_file = self.get_parameter("calibration_file").value
        self._calibration = self._load_calibration()

        # ---- State ----
        self._activated = False
        self._activate_cnt = 0
        self._try_activate = False
        self._activate_timer = None
        self._blocked = False
        self._mode = "normal"
        self._ee_pose = None          # latest from /ee_pose_visualize or TF
        self._last_pose = None        # for incremental delta calc
        self._last_poses = {}         # per-hand incremental state in dual mode
        self._p_err_buffer = Queue(maxsize=10)
        self._q_err_buffer = Queue(maxsize=10)
        self._axes = []
        # ---- Buttons ----
        self._buttons = {
            "X":  ButtonState("X"),
            "A":  ButtonState("A", self._a_press, self._a_release),
            "B":  ButtonState("B", self._b_press, self._b_release),
            "Y":  ButtonState("Y"),
            "LT": ButtonState("LT"),
            "RT": ButtonState("RT"),
            "LB": ButtonState("LB"),
            "RB": ButtonState("RB", self._rb_press, self._rb_release),
            "LS": ButtonState("LS"),
            "RS": ButtonState("RS"),
        }

        # ---- TF ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- Publisher ----
        self._pose_pub = self.create_publisher(PoseStamped, self._target_topic, 10)
        self._left_pose_pub = self.create_publisher(PoseStamped, self._left_target_topic, 10)
        self._right_pose_pub = self.create_publisher(PoseStamped, self._right_target_topic, 10)
        self._active_pub = self.create_publisher(Bool, self._active_topic, 1)
        self._left_active_pub = self.create_publisher(Bool, self._left_active_topic, 1)
        self._right_active_pub = self.create_publisher(Bool, self._right_active_topic, 1)
        self._left_gripper_pub = self.create_publisher(Float32, self._left_gripper_topic, 1)
        self._right_gripper_pub = self.create_publisher(Float32, self._right_gripper_topic, 1)

        # ---- Subscriber (VR joystick/buttons) ----
        self.create_subscription(Joy, self.get_parameter("joy_topic").value,
                                 self._joy_cb, 10)

        # ---- Control loop timer (50 Hz) ----
        period = 1.0 / max(self._pub_rate, 1.0)
        self._ctrl_timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"VR teleop ready | control={self._control_mode} | mode={self._mode} | "
            f"triple-press A to activate, B to switch mode, RB to block"
        )

    def _load_calibration(self):
        default = {
            "enabled": False,
            "mirror_convergence": False,
            "left": {
                "position_scale": [1.0, 1.0, 1.0],
                "position_offset": [0.0, 0.0, 0.0],
                "rotation_offset_quat": [0.0, 0.0, 0.0, 1.0],
            },
            "right": {
                "position_scale": [1.0, 1.0, 1.0],
                "position_offset": [0.0, 0.0, 0.0],
                "rotation_offset_quat": [0.0, 0.0, 0.0, 1.0],
            },
        }
        if not self._calibration_enabled:
            return default
        path = self._calibration_file
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(
                    get_package_share_directory("rm_dualarm"),
                    "config",
                    "vr_calibration.yaml",
                )
            except Exception:
                path = ""
        if not path or not os.path.exists(path):
            self.get_logger().warn("VR calibration file not found; using legacy mapping")
            return default
        with open(path, "r", encoding="utf-8") as f:
            loaded = yaml.safe_load(f) or {}
        default.update({k: v for k, v in loaded.items() if k in default})
        for side in ("left", "right"):
            merged = default[side]
            merged.update(loaded.get(side, {}) or {})
            default[side] = merged
        self.get_logger().info(f"Loaded VR calibration: {path}")
        return default

    # ==================================================================
    #  Joy callback — maps Quest controller indices to named buttons
    # ==================================================================

    def _joy_cb(self, msg: Joy):
        if len(msg.buttons) < 14:
            return
        self._axes = list(msg.axes)
        self._buttons["X"].update(msg.buttons[0])
        self._buttons["A"].update(msg.buttons[1])
        self._buttons["B"].update(msg.buttons[2])
        self._buttons["Y"].update(msg.buttons[3])
        self._buttons["LT"].update(msg.buttons[8])
        self._buttons["RT"].update(msg.buttons[9])
        self._buttons["LB"].update(msg.buttons[10])
        self._buttons["RB"].update(msg.buttons[11])
        self._buttons["LS"].update(msg.buttons[12])
        self._buttons["RS"].update(msg.buttons[13])
        self._publish_gripper_placeholders()

    # ==================================================================
    #  Button callbacks
    # ==================================================================

    def _a_press(self):
        self._try_activate = True
        if self._activate_timer is None:
            self._activate_timer = self.create_timer(
                3.0, self._activate_timeout
            )

    def _a_release(self):
        if self._try_activate:
            self._activate_cnt += 1
            self._try_activate = False
            if self._activate_cnt >= 3:
                self._activated = not self._activated
                self._activate_cnt = 0
                self._activate_timer.cancel()
                self._activate_timer = None
                state = "ACTIVE" if self._activated else "IDLE"
                self._publish_active(self._activated)
                self.get_logger().info(f"VR teleop {state}")

    def _activate_timeout(self):
        self._activate_cnt = 0
        self._try_activate = False
        self._activate_timer.cancel()
        self._activate_timer = None

    def _b_press(self):
        self._try_switch = True

    def _b_release(self):
        if getattr(self, "_try_switch", False):
            self._mode = "incremental" if self._mode == "normal" else "normal"
            self.get_logger().info(f"Mode → {self._mode}")
            self._try_switch = False

    def _rb_press(self):
        self._blocked = True

    def _rb_release(self):
        self._blocked = False

    # ==================================================================
    #  Main control loop (50 Hz timer)
    # ==================================================================

    def _control_loop(self):
        if not self._activated or self._blocked:
            self._last_pose = None
            self._last_poses.clear()
            return

        try:
            if self._control_mode == "dual":
                if self._mode == "normal":
                    self._dual_normal_control()
                elif self._mode == "incremental":
                    self._dual_incremental_control()
            elif self._mode == "normal":
                self._normal_control()
            elif self._mode == "incremental":
                self._incremental_control()
        except TransformException:
            return

    # ------------------------------------------------------------------
    def _publish_active(self, active):
        msg = Bool(data=active)
        if self._control_mode == "dual":
            self._left_active_pub.publish(msg)
            self._right_active_pub.publish(msg)
        else:
            self._active_pub.publish(msg)

    def _publish_gripper_placeholders(self):
        """Publish normalized trigger placeholders for future gripper control."""
        if len(self._axes) < 6:
            return
        # Provided mapping: axes[4]=LT, axes[5]=RT. Preserve as normalized commands.
        self._left_gripper_pub.publish(Float32(data=float(self._axes[4])))
        self._right_gripper_pub.publish(Float32(data=float(self._axes[5])))

    def _apply_calibration(self, side, pos, quat):
        if not self._calibration.get("enabled", False):
            return pos, quat
        cfg = self._calibration.get(side, {})
        scale = np.array(cfg.get("position_scale", [1.0, 1.0, 1.0]), dtype=float)
        offset = np.array(cfg.get("position_offset", [0.0, 0.0, 0.0]), dtype=float)
        pos = np.array(pos, dtype=float)
        if self._calibration.get("mirror_convergence", False) and side == "right":
            pos[1] = -pos[1]
        pos = pos * scale + offset

        q_offset = np.array(cfg.get("rotation_offset_quat", [0.0, 0.0, 0.0, 1.0]), dtype=float)
        quat = (R.from_quat(q_offset) * R.from_quat(quat)).as_quat()
        return pos, quat

    def _pose_from_hand(self, hand_frame, base_frame, y_offset=0.0, side="right"):
        now = Time()
        try:
            trans = self._tf_buffer.lookup_transform(
                self._vr_base, hand_frame, now,
                Duration(seconds=0.1),
            )
        except TransformException:
            self.get_logger().warn(
                f"TF lookup {self._vr_base}->{hand_frame} failed",
                throttle_duration_sec=2.0,
            )
            return None

        # Remap VR orientation to robot frame
        q_raw = np.array([
            trans.transform.rotation.x, trans.transform.rotation.y,
            trans.transform.rotation.z, trans.transform.rotation.w,
        ])
        r_orig = R.from_quat(q_raw).as_matrix()
        r_mapped = r_orig @ ORI_MAPPING
        q_mapped = R.from_matrix(r_mapped).as_quat()

        position = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y + y_offset,
            trans.transform.translation.z - (self._user_height / 2.0 - 0.2),
        ])
        position, q_mapped = self._apply_calibration(side, position, q_mapped)

        # Build PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = base_frame
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = q_mapped[0]
        pose.pose.orientation.y = q_mapped[1]
        pose.pose.orientation.z = q_mapped[2]
        pose.pose.orientation.w = q_mapped[3]
        return pose

    def _normal_control(self):
        """Absolute VR hand pose → robot target."""
        pose = self._pose_from_hand(self._hand_frame, self._base_frame, side="right")
        if pose is None:
            return
        self._pose_pub.publish(pose)

    def _dual_normal_control(self):
        left_pose = self._pose_from_hand(
            self._left_hand_frame, self._left_base_frame, self._dual_y_offset, "left")
        right_pose = self._pose_from_hand(
            self._right_hand_frame, self._right_base_frame, -self._dual_y_offset, "right")
        if left_pose is not None:
            self._left_pose_pub.publish(left_pose)
        if right_pose is not None:
            self._right_pose_pub.publish(right_pose)

    def _dual_incremental_control(self):
        """Dual-arm incremental mode is reserved; publish absolute dual targets for now."""
        self.get_logger().warn(
            "Dual incremental mode is not implemented; using dual normal control",
            throttle_duration_sec=2.0,
        )
        self._dual_normal_control()

    # ------------------------------------------------------------------
    def _incremental_control(self):
        """VR hand delta → accumulated robot target."""
        if self._ee_pose is None:
            self.get_logger().warn(
                "EE pose unknown — cannot do incremental control",
                throttle_duration_sec=2.0,
            )
            return

        now = Time()
        try:
            trans = self._tf_buffer.lookup_transform(
                self._vr_origin, self._hand_frame, now,
                Duration(seconds=0.1),
            )
        except TransformException:
            self.get_logger().warn("TF lookup vr_origin→hand_right failed", throttle_duration_sec=2.0)
            return

        if self._last_pose is None:
            self._last_pose = trans
            return

        # Position delta (clipped)
        p = np.array([trans.transform.translation.x,
                       trans.transform.translation.y,
                       trans.transform.translation.z])
        p_last = np.array([self._last_pose.transform.translation.x,
                            self._last_pose.transform.translation.y,
                            self._last_pose.transform.translation.z])
        p_err = p - p_last
        p_err = np.clip(p_err, -0.1, 0.1)

        # Orientation delta
        q = np.array([trans.transform.rotation.x,
                       trans.transform.rotation.y,
                       trans.transform.rotation.z,
                       trans.transform.rotation.w])
        q_last = np.array([self._last_pose.transform.rotation.x,
                            self._last_pose.transform.rotation.y,
                            self._last_pose.transform.rotation.z,
                            self._last_pose.transform.rotation.w])
        q_err = (R.from_quat(q) * R.from_quat(q_last).inv()).as_rotvec()
        q_err *= self._q_sensitivity

        # Filter errors with moving average
        self._p_err_buffer.put(p_err)
        self._q_err_buffer.put(q_err)
        if self._p_err_buffer.full():
            p_err = np.mean(list(self._p_err_buffer.queue), axis=0)
            self._p_err_buffer.get()
        if self._q_err_buffer.full():
            q_err = np.mean(list(self._q_err_buffer.queue), axis=0)
            self._q_err_buffer.get()

        # Compute next pose from current EE
        next_pose = PoseStamped()
        next_pose.header.stamp = self.get_clock().now().to_msg()
        next_pose.header.frame_id = self._ee_pose.header.frame_id
        next_pose.pose.position.x = (self._ee_pose.pose.position.x
                                     + p_err[0] * self._p_sensitivity)
        next_pose.pose.position.y = (self._ee_pose.pose.position.y
                                     + p_err[1] * self._p_sensitivity)
        next_pose.pose.position.z = (self._ee_pose.pose.position.z
                                     + p_err[2] * self._p_sensitivity)

        q_cur = np.array([self._ee_pose.pose.orientation.x,
                           self._ee_pose.pose.orientation.y,
                           self._ee_pose.pose.orientation.z,
                           self._ee_pose.pose.orientation.w])
        q_next = (R.from_rotvec(q_err) * R.from_quat(q_cur)).as_quat()
        next_pose.pose.orientation.x = q_next[0]
        next_pose.pose.orientation.y = q_next[1]
        next_pose.pose.orientation.z = q_next[2]
        next_pose.pose.orientation.w = q_next[3]

        self._pose_pub.publish(next_pose)
        self._last_pose = trans


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = VRTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
