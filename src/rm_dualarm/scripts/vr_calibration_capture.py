#!/usr/bin/env python3
"""Capture dual-arm VR calibration from a prescribed operator pose."""

import os
import yaml

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener, TransformException


ORI_MAPPING = np.array([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0],
])


def _transform_to_pose(transform):
    p = transform.transform.translation
    q = transform.transform.rotation
    return np.array([p.x, p.y, p.z], dtype=float), np.array([q.x, q.y, q.z, q.w], dtype=float)


def _map_vr_quat_to_robot(quat):
    return R.from_matrix(R.from_quat(quat).as_matrix() @ ORI_MAPPING).as_quat()


def _average_quat(quats):
    rotations = R.from_quat(np.asarray(quats, dtype=float))
    return rotations.mean().as_quat()


class VRCalibrationCapture(Node):
    """Collects a short TF window and writes a schema v2 calibration YAML."""

    def __init__(self):
        super().__init__("vr_calibration_capture")
        self.declare_parameter("vr_base_frame", "vr_base")
        self.declare_parameter("vr_origin_frame", "vr_origin")
        self.declare_parameter("headset_frame", "headset")
        self.declare_parameter("left_vr_hand_frame", "hand_left")
        self.declare_parameter("right_vr_hand_frame", "hand_right")
        self.declare_parameter("left_base_frame", "left_base_link")
        self.declare_parameter("right_base_frame", "right_base_link")
        self.declare_parameter("left_ee_frame", "left_Link7")
        self.declare_parameter("right_ee_frame", "right_Link7")
        self.declare_parameter("output_file", "/tmp/vr_calibration.yaml")
        self.declare_parameter("joy_topic", "/quest/joystick")
        self.declare_parameter("sample_duration", 2.0)
        self.declare_parameter("sample_rate", 50.0)
        self.declare_parameter("mirror_convergence", False)

        self._vr_base = self.get_parameter("vr_base_frame").value
        self._vr_origin = self.get_parameter("vr_origin_frame").value
        self._headset = self.get_parameter("headset_frame").value
        self._left_hand = self.get_parameter("left_vr_hand_frame").value
        self._right_hand = self.get_parameter("right_vr_hand_frame").value
        self._left_base = self.get_parameter("left_base_frame").value
        self._right_base = self.get_parameter("right_base_frame").value
        self._left_ee = self.get_parameter("left_ee_frame").value
        self._right_ee = self.get_parameter("right_ee_frame").value
        self._output_file = self.get_parameter("output_file").value
        self._sample_duration = float(self.get_parameter("sample_duration").value)
        self._sample_rate = float(self.get_parameter("sample_rate").value)
        self._mirror_convergence = bool(self.get_parameter("mirror_convergence").value)

        self._last_y = False
        self._sampling = False
        self._samples = []
        self._sample_timer = None
        self._sample_start = None

        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._joy_cb, 10)

        self.get_logger().info(
            "Stand upright, upper arms against body, forearms at 90 deg and "
            "parallel to ground, controllers parallel to ground. Press Y to sample."
        )

    def _lookup_pose(self, parent, child):
        transform = self._tf.lookup_transform(
            parent, child, Time(), Duration(seconds=0.2))
        return _transform_to_pose(transform)

    def _joy_cb(self, msg):
        y_pressed = len(msg.buttons) > 3 and bool(msg.buttons[3])
        if y_pressed and not self._last_y and not self._sampling:
            self._start_sampling()
        self._last_y = y_pressed

    def _start_sampling(self):
        self._sampling = True
        self._samples = []
        self._sample_start = self.get_clock().now()
        period = 1.0 / max(self._sample_rate, 1.0)
        self._sample_timer = self.create_timer(period, self._sample_once)
        self.get_logger().info(
            f"Sampling calibration for {self._sample_duration:.1f}s ..."
        )

    def _sample_once(self):
        elapsed = (self.get_clock().now() - self._sample_start).nanoseconds / 1e9
        if elapsed >= self._sample_duration:
            self._finish_sampling()
            return
        try:
            headset_p, headset_q = self._lookup_pose(self._vr_origin, self._headset)
            left_hand_p, left_hand_q = self._lookup_pose(self._vr_base, self._left_hand)
            right_hand_p, right_hand_q = self._lookup_pose(self._vr_base, self._right_hand)
            left_ee_p, left_ee_q = self._lookup_pose(self._left_base, self._left_ee)
            right_ee_p, right_ee_q = self._lookup_pose(self._right_base, self._right_ee)
        except TransformException as exc:
            self.get_logger().warn(f"Skipping calibration sample: {exc}")
            return

        self._samples.append({
            "headset_p": headset_p,
            "headset_q": headset_q,
            "left_hand_p": left_hand_p,
            "left_hand_q": _map_vr_quat_to_robot(left_hand_q),
            "right_hand_p": right_hand_p,
            "right_hand_q": _map_vr_quat_to_robot(right_hand_q),
            "left_ee_p": left_ee_p,
            "left_ee_q": left_ee_q,
            "right_ee_p": right_ee_p,
            "right_ee_q": right_ee_q,
        })

    def _mean_vec(self, key):
        return np.mean([sample[key] for sample in self._samples], axis=0)

    def _mean_quat(self, key):
        return _average_quat([sample[key] for sample in self._samples])

    def _pose_yaml(self, position, quat):
        return {
            "position": [float(v) for v in position],
            "orientation_quat": [float(v) for v in quat],
        }

    def _finish_sampling(self):
        if self._sample_timer is not None:
            self._sample_timer.cancel()
            self._sample_timer = None
        self._sampling = False

        if len(self._samples) < max(5, int(self._sample_rate * 0.5)):
            self.get_logger().error(
                f"Calibration failed: only {len(self._samples)} valid samples"
            )
            return

        headset_p = self._mean_vec("headset_p")
        data = {
            "schema_version": 2,
            "enabled": True,
            "description": (
                "Operator upright; upper arms down against body; forearms at 90 deg; "
                "controllers parallel to ground. Robot arms are in standby pose."
            ),
            "sample_count": len(self._samples),
            "frames": {
                "vr_origin": self._vr_origin,
                "vr_base": self._vr_base,
                "headset": self._headset,
                "left_hand": self._left_hand,
                "right_hand": self._right_hand,
                "left_base": self._left_base,
                "right_base": self._right_base,
                "left_ee": self._left_ee,
                "right_ee": self._right_ee,
            },
            "operator_center": {
                "height_from_ground": float(headset_p[2] * 0.5),
            },
            "mirror_convergence": self._mirror_convergence,
            "left": {
                "position_scale": [1.0, 1.0, 1.0],
                "rotation_scale": 1.0,
                "vr_hand_standby": self._pose_yaml(
                    self._mean_vec("left_hand_p"), self._mean_quat("left_hand_q")),
                "robot_ee_standby": self._pose_yaml(
                    self._mean_vec("left_ee_p"), self._mean_quat("left_ee_q")),
            },
            "right": {
                "position_scale": [1.0, 1.0, 1.0],
                "rotation_scale": 1.0,
                "vr_hand_standby": self._pose_yaml(
                    self._mean_vec("right_hand_p"), self._mean_quat("right_hand_q")),
                "robot_ee_standby": self._pose_yaml(
                    self._mean_vec("right_ee_p"), self._mean_quat("right_ee_q")),
            },
        }

        os.makedirs(os.path.dirname(os.path.abspath(self._output_file)), exist_ok=True)
        with open(self._output_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(
            f"Saved VR calibration to {self._output_file} ({len(self._samples)} samples)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VRCalibrationCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
