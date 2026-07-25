#!/usr/bin/env python3
"""VR hand-tracking teleoperation node (ROS 2 port from noetic).

Subscribes to /quest/joystick (Joy) and TF from the VR headset to map
the user's hand pose directly to the robot end-effector target.

Control mode:
  normal  — absolute VR hand pose → robot target

Activation:  triple-press A
Block:       hold RB
Recording:   press B to start/stop rosbag recording

Output: PoseStamped on /target_pose (consumed by servo_pose_tracking_demo).
"""

from queue import Queue
import os
import signal
import subprocess
import time
import yaml

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from rm_ros_interfaces.msg import Gripperset
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, Float32, String
from tf2_ros import Buffer, TransformListener, TransformException


# Remap VR coordinate frame to robot frame.
# VR: X=right, Y=up, Z=backward  →  Robot: X=forward, Y=left, Z=up
ORI_MAPPING = np.array([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0],
])

_PLANNING_GROUP = "rm_group"
_EXPECTED_JOINTS = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
]
_DUAL_TARGETS = [
    ("left_rm_group", "left_"),
    ("right_rm_group", "right_"),
]
_SINGLE_RECORD_TOPICS = [
    "/joint_states",
    "/target_pose",
    "/tf",
    "/tf_static",
    "/teleop_active",
    "/servo_node/delta_twist_cmds",
    "/servo_node/status",
    "/rm_group_controller/joint_trajectory",
    "/servo_bridge/joint_trajectory_in",
    "/rm_driver/movej_canfd_cmd",
    "/rm_driver/set_gripper_position_cmd",
    "/camera/image_raw",
    "/wrist_camera/color/image_raw",
]
_DUAL_RECORD_TOPICS = [
    "/joint_states",
    "/left/target_pose",
    "/right/target_pose",
    "/tf",
    "/tf_static",
    "/left_servo_node/delta_twist_cmds",
    "/right_servo_node/delta_twist_cmds",
    "/left_rm_group_controller/joint_trajectory",
    "/right_rm_group_controller/joint_trajectory",
    "/left_servo_bridge/joint_trajectory_in",
    "/right_servo_bridge/joint_trajectory_in",
    "/left/rm_driver/movej_canfd_cmd",
    "/right/rm_driver/movej_canfd_cmd",
    "/left/wrist_camera/color/image_raw",
    "/right/wrist_camera/color/image_raw",
]


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
        self.declare_parameter("servo_bridge_stop_topic", "/servo_bridge/stop")
        self.declare_parameter("left_gripper_topic", "/left/gripper_cmd")
        self.declare_parameter("right_gripper_topic", "/right/gripper_cmd")
        self.declare_parameter("left_gripper_driver_topic", "/left/rm_driver/set_gripper_position_cmd")
        self.declare_parameter("right_gripper_driver_topic", "/right/rm_driver/set_gripper_position_cmd")
        self.declare_parameter("gripper_driver_topic", "/rm_driver/set_gripper_position_cmd")
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
        self.declare_parameter("mirror_mode", False)
        self.declare_parameter("standby_pose_file", "")
        self.declare_parameter("record_output_prefix", "bags/vr_teleop")
        self.declare_parameter("recording_active_topic", "/vr_teleop/recording_active")
        self.declare_parameter("recording_status_topic", "/vr_teleop/recording_status")
        self.declare_parameter("recording_status_period", 5.0)
        self.declare_parameter("target_filter_enabled", True)
        self.declare_parameter("target_filter_alpha", 0.6)

        self._control_mode = self.get_parameter("control_mode").value
        self._target_topic = self.get_parameter("target_topic").value
        self._left_target_topic = self.get_parameter("left_target_topic").value
        self._right_target_topic = self.get_parameter("right_target_topic").value
        self._active_topic = self.get_parameter("active_topic").value
        self._left_active_topic = self.get_parameter("left_active_topic").value
        self._right_active_topic = self.get_parameter("right_active_topic").value
        self._servo_bridge_stop_topic = self.get_parameter("servo_bridge_stop_topic").value
        self._left_gripper_topic = self.get_parameter("left_gripper_topic").value
        self._right_gripper_topic = self.get_parameter("right_gripper_topic").value
        self._left_gripper_driver_topic = self.get_parameter("left_gripper_driver_topic").value
        self._right_gripper_driver_topic = self.get_parameter("right_gripper_driver_topic").value
        self._gripper_driver_topic = self.get_parameter("gripper_driver_topic").value
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
        self._mirror_mode = self.get_parameter("mirror_mode").value
        self._standby_pose_file = self.get_parameter("standby_pose_file").value
        self._record_output_prefix = self.get_parameter("record_output_prefix").value
        self._recording_active_topic = self.get_parameter("recording_active_topic").value
        self._recording_status_topic = self.get_parameter("recording_status_topic").value
        self._recording_status_period = float(
            self.get_parameter("recording_status_period").value)
        self._target_filter_enabled = self.get_parameter("target_filter_enabled").value
        self._target_filter_alpha = self.get_parameter("target_filter_alpha").value
        self._target_filter_alpha = max(0.0, min(1.0, self._target_filter_alpha))
        self._calibration = self._load_calibration()

        # ---- State ----
        self._activated = False
        self._a_held = False
        self._activate_cnt = 0
        self._try_activate = False
        self._activate_timer = None
        self._standby_hold_timer = None
        self._standby_hold_consumed = False
        self._standby_in_progress = False
        self._standby_targets = []
        self._standby_target_index = 0
        self._blocked = False
        self._mode = "normal"
        self._mirror_x_home = {}       # {"left": x, "right": x} — X pivot per arm
        self._ee_pose = None          # latest from /ee_pose_visualize or TF
        self._last_pose = None        # for incremental delta calc
        self._last_poses = {}         # per-hand incremental state in dual mode
        self._p_err_buffer = Queue(maxsize=10)
        self._q_err_buffer = Queue(maxsize=10)
        self._axes = []
        self._record_proc = None
        self._record_output = None
        self._record_started_at = None
        self._record_button_armed = False
        self._target_filters = {}
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

        # ---- MoveIt action client for idle standby recovery ----
        self._move_action = ActionClient(self, MoveGroup, "/move_action")

        # ---- Publisher ----
        self._pose_pub = self.create_publisher(PoseStamped, self._target_topic, 10)
        self._left_pose_pub = self.create_publisher(PoseStamped, self._left_target_topic, 10)
        self._right_pose_pub = self.create_publisher(PoseStamped, self._right_target_topic, 10)
        self._active_pub = self.create_publisher(Bool, self._active_topic, 1)
        self._left_active_pub = self.create_publisher(Bool, self._left_active_topic, 1)
        self._right_active_pub = self.create_publisher(Bool, self._right_active_topic, 1)
        self._servo_bridge_stop_pub = self.create_publisher(
            Empty, self._servo_bridge_stop_topic, 1)
        self._left_gripper_pub = self.create_publisher(Float32, self._left_gripper_topic, 1)
        self._right_gripper_pub = self.create_publisher(Float32, self._right_gripper_topic, 1)
        self._left_gripper_driver_pub = self.create_publisher(
            Gripperset, self._left_gripper_driver_topic, 1)
        self._right_gripper_driver_pub = self.create_publisher(
            Gripperset, self._right_gripper_driver_topic, 1)
        self._gripper_driver_pub = self.create_publisher(
            Gripperset, self._gripper_driver_topic, 1)
        self._recording_active_pub = self.create_publisher(
            Bool, self._recording_active_topic, 1)
        self._recording_status_pub = self.create_publisher(
            String, self._recording_status_topic, 10)
        self._last_gripper_closed = None         # debounce for single mode
        self._last_left_gripper_closed = None    # debounce for dual mode
        self._last_right_gripper_closed = None

        # ---- Subscriber (VR joystick/buttons) ----
        self.create_subscription(Joy, self.get_parameter("joy_topic").value,
                                 self._joy_cb, 10)

        # ---- Control loop timer (50 Hz) ----
        period = 1.0 / max(self._pub_rate, 1.0)
        self._ctrl_timer = self.create_timer(period, self._control_loop)
        self._recording_status_timer = None
        if self._recording_status_period > 0.0:
            self._recording_status_timer = self.create_timer(
                max(self._recording_status_period, 0.5),
                self._recording_status_timer_cb,
            )
        self._publish_recording_status("recording_idle", False)

        self.get_logger().info(
            f"VR teleop ready | control={self._control_mode} | mode={self._mode}"
            f"{' | MIRROR' if self._mirror_mode else ''}"
            f" | target_filter={'on' if self._target_filter_enabled else 'off'}"
            f" (alpha={self._target_filter_alpha:.2f})"
            f" | triple-press A to activate, hold A 3s in IDLE for standby"
            f", B to start/stop recording, RB to block"
            f" | recording_status={self._recording_status_topic}"
        )

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
        path = self._standby_pose_file or self._default_standby_pose_file()
        if not os.path.exists(path):
            raise RuntimeError(f"standby pose file not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        joints = data.get("joints", data)
        missing = [name for name in _EXPECTED_JOINTS if name not in joints]
        if missing:
            raise RuntimeError(
                f"Standby pose file {path} missing joints: {', '.join(missing)}"
            )
        self.get_logger().info(f"Loaded standby pose: {path}")
        return {name: float(joints[name]) for name in _EXPECTED_JOINTS}

    def _make_standby_targets(self):
        base_joints = self._load_standby_joints()
        if self._control_mode == "dual":
            return [
                (
                    group,
                    {
                        f"{prefix}{joint}": position
                        for joint, position in base_joints.items()
                    },
                )
                for group, prefix in _DUAL_TARGETS
            ]
        return [(_PLANNING_GROUP, dict(base_joints))]

    def _load_calibration(self):
        default = {
            "schema_version": 1,
            "enabled": False,
            "mirror_convergence": False,
            "left": {
                "position_scale": [1.0, 1.0, 1.0],
                "position_offset": [0.0, 0.0, 0.0],
                "rotation_offset_quat": [0.0, 0.0, 0.0, 1.0],
                "rotation_scale": 1.0,
            },
            "right": {
                "position_scale": [1.0, 1.0, 1.0],
                "position_offset": [0.0, 0.0, 0.0],
                "rotation_offset_quat": [0.0, 0.0, 0.0, 1.0],
                "rotation_scale": 1.0,
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
        for key, value in loaded.items():
            if key in default and key not in ("left", "right"):
                default[key] = value
        for side in ("left", "right"):
            merged = default[side]
            merged.update(loaded.get(side, {}) or {})
            default[side] = merged
        self.get_logger().info(
            f"Loaded VR calibration: {path} (schema v{default.get('schema_version', 1)})"
        )
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
        self._a_held = True
        self._standby_hold_consumed = False
        self._try_activate = True
        if self._activate_timer is None:
            self._activate_timer = self.create_timer(
                3.0, self._activate_timeout
            )
        if not self._activated and self._standby_hold_timer is None:
            self._standby_hold_timer = self.create_timer(
                3.0, self._standby_hold_timeout
            )

    def _a_release(self):
        self._a_held = False
        if self._standby_hold_timer is not None:
            self._standby_hold_timer.cancel()
            self._standby_hold_timer = None
        if self._standby_hold_consumed:
            self._try_activate = False
            self._standby_hold_consumed = False
            return
        if self._try_activate:
            self._activate_cnt += 1
            self._try_activate = False
            if self._activate_cnt >= 3:
                self._activated = not self._activated
                self._activate_cnt = 0
                self._activate_timer.cancel()
                self._activate_timer = None
                if not self._activated:
                    self._mirror_x_home.clear()
                    self._reset_target_filters()
                state = "ACTIVE" if self._activated else "IDLE"
                self._publish_active(self._activated)
                self.get_logger().info(f"VR teleop {state}")

    def _activate_timeout(self):
        self._activate_cnt = 0
        self._try_activate = False
        self._activate_timer.cancel()
        self._activate_timer = None

    def _standby_hold_timeout(self):
        if self._standby_hold_timer is not None:
            self._standby_hold_timer.cancel()
            self._standby_hold_timer = None
        if self._activated or not self._a_held:
            return

        self._standby_hold_consumed = True
        self._activate_cnt = 0
        self._try_activate = False
        if self._activate_timer is not None:
            self._activate_timer.cancel()
            self._activate_timer = None
        self._last_pose = None
        self._last_poses.clear()
        self._mirror_x_home.clear()
        self._reset_target_filters()
        self._publish_active(False)
        self._stop_servo_bridge()
        self._send_standby_goals()

    def _b_press(self):
        self._record_button_armed = True

    def _b_release(self):
        if self._record_button_armed:
            self._toggle_recording()
            self._record_button_armed = False

    def _rb_press(self):
        self._blocked = True
        self._reset_target_filters()

    def _rb_release(self):
        self._blocked = False

    # ==================================================================
    #  Main control loop (50 Hz timer)
    # ==================================================================

    def _control_loop(self):
        if not self._activated or self._blocked:
            self._last_pose = None
            self._last_poses.clear()
            self._reset_target_filters()
            return

        try:
            if self._control_mode == "dual":
                self._dual_normal_control()
            else:
                self._normal_control()
        except TransformException:
            return

    # ------------------------------------------------------------------
    def _record_topics(self):
        if self._control_mode == "dual":
            return list(_DUAL_RECORD_TOPICS)
        return list(_SINGLE_RECORD_TOPICS)

    def _make_record_output(self):
        prefix = self._record_output_prefix or "bags/vr_teleop"
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        mode = "dual" if self._control_mode == "dual" else "single"
        return f"{prefix}_{mode}_{timestamp}"

    def _toggle_recording(self):
        if self._record_proc is not None and self._record_proc.poll() is None:
            self._stop_recording()
        else:
            self._start_recording()

    def _recording_elapsed(self):
        if self._record_started_at is None:
            return 0.0
        return time.monotonic() - self._record_started_at

    def _publish_recording_status(self, text, active):
        self._recording_active_pub.publish(Bool(data=active))
        msg = String()
        msg.data = text
        self._recording_status_pub.publish(msg)

    def _recording_status_timer_cb(self):
        if self._record_proc is None or self._record_proc.poll() is not None:
            return
        output = self._record_output or "(unknown output)"
        elapsed = self._recording_elapsed()
        status = f"recording_active output={output} elapsed={elapsed:.1f}s"
        self._publish_recording_status(status, True)
        self.get_logger().info(
            f"ROSBAG RECORDING ACTIVE ({elapsed:.0f}s): {output}"
        )

    def _start_recording(self):
        if self._record_proc is not None and self._record_proc.poll() is None:
            output = self._record_output or "(unknown output)"
            self.get_logger().warn(f"Rosbag recording is already running: {output}")
            self._publish_recording_status(
                f"recording_active output={output} elapsed={self._recording_elapsed():.1f}s",
                True,
            )
            return

        output = self._make_record_output()
        output_dir = os.path.dirname(output)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        cmd = ["ros2", "bag", "record", "-o", output] + self._record_topics()
        try:
            self._record_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as exc:
            self._record_proc = None
            self._record_output = None
            self._record_started_at = None
            self._publish_recording_status(f"recording_error error={exc}", False)
            self.get_logger().error(f"Failed to start rosbag recording: {exc}")
            return

        self._record_output = output
        self._record_started_at = time.monotonic()
        self._publish_recording_status(
            f"recording_started output={output} elapsed=0.0s",
            True,
        )
        self.get_logger().warn(f"*** ROSBAG RECORDING STARTED *** {output}")

    def _stop_recording(self):
        if self._record_proc is None:
            return

        proc = self._record_proc
        output = self._record_output
        duration = self._recording_elapsed()
        if proc.poll() is not None:
            self.get_logger().warn(
                f"Rosbag recording already stopped: {output}"
            )
            self._record_proc = None
            self._record_output = None
            self._record_started_at = None
            self._publish_recording_status(
                f"recording_stopped output={output} duration={duration:.1f}s",
                False,
            )
            return

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                "Rosbag did not stop after SIGINT; terminating process"
            )
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    "Rosbag did not terminate; killing process"
                )
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2.0)
        except OSError as exc:
            self.get_logger().warn(f"Failed to stop rosbag process: {exc}")
        finally:
            self._record_proc = None
            self._record_output = None
            self._record_started_at = None

        self._publish_recording_status(
            f"recording_stopped output={output} duration={duration:.1f}s",
            False,
        )
        self.get_logger().warn(
            f"*** ROSBAG RECORDING STOPPED *** {output} ({duration:.1f}s)"
        )

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()

    def _publish_active(self, active):
        msg = Bool(data=active)
        if self._control_mode == "dual":
            self._left_active_pub.publish(msg)
            self._right_active_pub.publish(msg)
        else:
            self._active_pub.publish(msg)

    def _reset_target_filters(self):
        self._target_filters.clear()

    def _stop_servo_bridge(self):
        self._servo_bridge_stop_pub.publish(Empty())
        self.get_logger().info(
            f"Requested Servo bridge stop on {self._servo_bridge_stop_topic}"
        )

    def _send_standby_goals(self):
        if self._standby_in_progress:
            self.get_logger().warn("Standby request ignored; MoveIt goal already running")
            return
        if not self._move_action.wait_for_server(timeout_sec=0.1):
            self.get_logger().error(
                "Cannot send standby goal: /move_action is not available"
            )
            return
        try:
            self._standby_targets = self._make_standby_targets()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return

        self._standby_in_progress = True
        self._standby_target_index = 0
        self.get_logger().info(
            f"Sending standby MoveIt goal(s) ({len(self._standby_targets)} target(s))"
        )
        self._send_next_standby_goal()

    def _send_next_standby_goal(self):
        if self._standby_target_index >= len(self._standby_targets):
            self._standby_in_progress = False
            self.get_logger().info("Standby pose reached")
            return

        group_name, joint_positions = self._standby_targets[self._standby_target_index]
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        for joint_name, position in joint_positions.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.replan_delay = 1.0

        self.get_logger().info(f"[{group_name}] sending standby goal")
        future = self._move_action.send_goal_async(goal)
        future.add_done_callback(self._standby_goal_response_cb)

    def _standby_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._standby_in_progress = False
            self.get_logger().error("Standby goal rejected by move_group")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._standby_result_cb)

    def _standby_result_cb(self, future):
        result = future.result().result
        group_name, _ = self._standby_targets[self._standby_target_index]
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info(f"[{group_name}] standby pose reached")
            self._standby_target_index += 1
            self._send_next_standby_goal()
        else:
            self._standby_in_progress = False
            self.get_logger().error(
                f"[{group_name}] standby FAILED: error_code={result.error_code.val}"
            )

    def _publish_gripper_placeholders(self):
        """Map LT/RT triggers → gripper open (900) / close (100).

        Trigger value > 0.5  →  close (position=100)
        Trigger value ≤ 0.5  →  open  (position=900)

        Commands are only published on state change to avoid spamming."""
        if len(self._axes) < 6:
            return
        left_val = float(self._axes[4])
        right_val = float(self._axes[5])

        # Keep Float32 placeholders for backward compat
        self._left_gripper_pub.publish(Float32(data=left_val))
        self._right_gripper_pub.publish(Float32(data=right_val))

        if self._control_mode == "single":
            # Single arm: RT trigger → gripper on /rm_driver/set_gripper_position_cmd
            close = right_val > 0.5
            if close != self._last_gripper_closed:
                self._last_gripper_closed = close
                pos = 100 if close else 900
                self._gripper_driver_pub.publish(
                    Gripperset(position=pos, block=False, timeout=0))
                self.get_logger().info(
                    f"Gripper → {'CLOSE' if close else 'OPEN'} (pos={pos})")
        else:
            # Dual arm: LT → left, RT → right (swapped in mirror mode)
            left_close = left_val > 0.5
            right_close = right_val > 0.5

            if self._mirror_mode:
                left_pub, right_pub = self._right_gripper_driver_pub, self._left_gripper_driver_pub
            else:
                left_pub, right_pub = self._left_gripper_driver_pub, self._right_gripper_driver_pub

            if left_close != self._last_left_gripper_closed:
                self._last_left_gripper_closed = left_close
                pos = 100 if left_close else 900
                left_pub.publish(Gripperset(position=pos, block=False, timeout=0))
                self.get_logger().info(
                    f"Left trigger → {'CLOSE' if left_close else 'OPEN'} (pos={pos})")

            if right_close != self._last_right_gripper_closed:
                self._last_right_gripper_closed = right_close
                pos = 100 if right_close else 900
                right_pub.publish(Gripperset(position=pos, block=False, timeout=0))
                self.get_logger().info(
                    f"Right trigger → {'CLOSE' if right_close else 'OPEN'} (pos={pos})")

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

    def _mapped_hand_pose(self, trans):
        q_raw = np.array([
            trans.transform.rotation.x, trans.transform.rotation.y,
            trans.transform.rotation.z, trans.transform.rotation.w,
        ])
        r_orig = R.from_quat(q_raw).as_matrix()
        r_mapped = r_orig @ ORI_MAPPING
        q_mapped = R.from_matrix(r_mapped).as_quat()
        position = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        ], dtype=float)
        return position, q_mapped

    def _mirror_orientation(self, quat):
        """Mirror orientation for face-to-face teleop: negate roll & yaw, keep pitch."""
        r = R.from_quat(quat)
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        mirrored = R.from_euler('xyz', [-roll, pitch, -yaw], degrees=False)
        return mirrored.as_quat()

    def _apply_mirror_x(self, pose, side):
        """Mirror X delta around the activation home position (face-to-face).

        Captures the target X on first call per activation per side,
        then reflects subsequent X movement: hand forward → target backward.
        """
        x = pose.pose.position.x
        home = self._mirror_x_home.get(side)
        if home is None:
            self._mirror_x_home[side] = x
            home = x
        pose.pose.position.x = home - (x - home)  # = 2*home - x

    def _make_pose_msg(self, base_frame, position, quat):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = base_frame
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose

    def _filtered_pose(self, key, pose):
        if not self._target_filter_enabled:
            return pose

        a = self._target_filter_alpha
        raw_p = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ], dtype=float)
        raw_q = np.array([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ], dtype=float)
        q_norm = np.linalg.norm(raw_q)
        if q_norm < 1e-9:
            raw_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        else:
            raw_q = raw_q / q_norm

        prev = self._target_filters.get(key)
        if prev is None:
            filt_p = raw_p
            filt_q = raw_q
        else:
            prev_p, prev_q = prev
            filt_p = a * raw_p + (1.0 - a) * prev_p
            delta_r = R.from_quat(raw_q) * R.from_quat(prev_q).inv()
            filt_q = (R.from_rotvec(delta_r.as_rotvec() * a)
                      * R.from_quat(prev_q)).as_quat()

        self._target_filters[key] = (filt_p, filt_q)
        return self._make_pose_msg(pose.header.frame_id, filt_p, filt_q)

    def _pose_from_calibrated_hand(self, hand_frame, base_frame, side):
        cfg = self._calibration.get(side, {})
        hand_ref = cfg.get("vr_hand_standby") or {}
        ee_ref = cfg.get("robot_ee_standby") or {}
        try:
            hand_ref_p = np.array(hand_ref["position"], dtype=float)
            hand_ref_q = np.array(hand_ref["orientation_quat"], dtype=float)
            ee_ref_p = np.array(ee_ref["position"], dtype=float)
            ee_ref_q = np.array(ee_ref["orientation_quat"], dtype=float)
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                f"VR calibration schema v2 missing {side} standby pose; using legacy mapping",
                throttle_duration_sec=2.0,
            )
            return self._pose_from_hand(hand_frame, base_frame, side=side)

        trans = self._tf_buffer.lookup_transform(
            self._vr_base, hand_frame, Time(), Duration(seconds=0.1))
        hand_p, hand_q = self._mapped_hand_pose(trans)

        delta_p = hand_p - hand_ref_p
        scale = np.array(cfg.get("position_scale", [1.0, 1.0, 1.0]), dtype=float)
        target_p = ee_ref_p + delta_p * scale

        delta_r = R.from_quat(hand_q) * R.from_quat(hand_ref_q).inv()
        rotation_scale = float(cfg.get("rotation_scale", 1.0))
        if rotation_scale != 1.0:
            delta_r = R.from_rotvec(delta_r.as_rotvec() * rotation_scale)
        target_q = (delta_r * R.from_quat(ee_ref_q)).as_quat()

        if self._mirror_mode:
            target_p[1] = -target_p[1]
            target_q = self._mirror_orientation(target_q)

        return self._make_pose_msg(base_frame, target_p, target_q)

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
        hand_position, q_mapped = self._mapped_hand_pose(trans)

        position = np.array([
            hand_position[0],
            hand_position[1] + y_offset,
            hand_position[2] - (self._user_height / 2.0 - 0.2),
        ])
        position, q_mapped = self._apply_calibration(side, position, q_mapped)

        if self._mirror_mode:
            position[1] = -position[1]
            q_mapped = self._mirror_orientation(q_mapped)

        return self._make_pose_msg(base_frame, position, q_mapped)

    def _normal_control(self):
        """Absolute VR hand pose → robot target."""
        if self._calibration.get("enabled") and self._calibration.get("schema_version") == 2:
            pose = self._pose_from_calibrated_hand(self._hand_frame, self._base_frame, "right")
        else:
            pose = self._pose_from_hand(self._hand_frame, self._base_frame, side="right")
        if pose is None:
            return

        if self._mirror_mode:
            self._apply_mirror_x(pose, "right")

        self._pose_pub.publish(self._filtered_pose("right", pose))

    def _dual_normal_control(self):
        if self._mirror_mode:
            # Face-to-face: right hand → left arm, left hand → right arm
            if self._calibration.get("enabled") and self._calibration.get("schema_version") == 2:
                left_pose = self._pose_from_calibrated_hand(
                    self._right_hand_frame, self._left_base_frame, "right")
                right_pose = self._pose_from_calibrated_hand(
                    self._left_hand_frame, self._right_base_frame, "left")
            else:
                left_pose = self._pose_from_hand(
                    self._right_hand_frame, self._left_base_frame, -self._dual_y_offset, "right")
                right_pose = self._pose_from_hand(
                    self._left_hand_frame, self._right_base_frame, self._dual_y_offset, "left")
        else:
            if self._calibration.get("enabled") and self._calibration.get("schema_version") == 2:
                left_pose = self._pose_from_calibrated_hand(
                    self._left_hand_frame, self._left_base_frame, "left")
                right_pose = self._pose_from_calibrated_hand(
                    self._right_hand_frame, self._right_base_frame, "right")
            else:
                left_pose = self._pose_from_hand(
                    self._left_hand_frame, self._left_base_frame, self._dual_y_offset, "left")
                right_pose = self._pose_from_hand(
                    self._right_hand_frame, self._right_base_frame, -self._dual_y_offset, "right")

        if self._mirror_mode:
            if left_pose is not None:
                self._apply_mirror_x(left_pose, "left")
            if right_pose is not None:
                self._apply_mirror_x(right_pose, "right")

        if left_pose is not None:
            self._left_pose_pub.publish(self._filtered_pose("left", left_pose))
        if right_pose is not None:
            self._right_pose_pub.publish(self._filtered_pose("right", right_pose))

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
