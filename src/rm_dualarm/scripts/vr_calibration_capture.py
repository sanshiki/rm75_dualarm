#!/usr/bin/env python3
"""Capture a simple dual-hand VR calibration YAML from live TF."""

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener, TransformException


class VRCalibrationCapture(Node):
    def __init__(self):
        super().__init__("vr_calibration_capture")
        self.declare_parameter("vr_base_frame", "vr_base")
        self.declare_parameter("left_vr_hand_frame", "hand_left")
        self.declare_parameter("right_vr_hand_frame", "hand_right")
        self.declare_parameter("output_file", "/tmp/vr_calibration.yaml")
        self.declare_parameter("joy_topic", "/quest/joystick")
        self._vr_base = self.get_parameter("vr_base_frame").value
        self._left_hand = self.get_parameter("left_vr_hand_frame").value
        self._right_hand = self.get_parameter("right_vr_hand_frame").value
        self._output_file = self.get_parameter("output_file").value
        self._last_y = False
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._joy_cb, 10)
        self.get_logger().info("Hold neutral pose and press Y to save calibration YAML")

    def _lookup(self, frame):
        t = self._tf.lookup_transform(
            self._vr_base, frame, Time(), Duration(seconds=0.2))
        p = t.transform.translation
        r = t.transform.rotation
        return [p.x, p.y, p.z], [r.x, r.y, r.z, r.w]

    def _joy_cb(self, msg):
        y_pressed = len(msg.buttons) > 3 and bool(msg.buttons[3])
        if not y_pressed or self._last_y:
            self._last_y = y_pressed
            return
        self._last_y = y_pressed
        try:
            left_p, _ = self._lookup(self._left_hand)
            right_p, _ = self._lookup(self._right_hand)
        except TransformException as exc:
            self.get_logger().error(f"Cannot capture calibration: {exc}")
            return

        data = {
            "enabled": True,
            "mirror_convergence": True,
            "left": {
                "position_scale": [1.0, 1.0, 1.0],
                "position_offset": [-left_p[0], -left_p[1], -left_p[2]],
                "rotation_offset_quat": [0.0, 0.0, 0.0, 1.0],
            },
            "right": {
                "position_scale": [1.0, 1.0, 1.0],
                "position_offset": [-right_p[0], right_p[1], -right_p[2]],
                "rotation_offset_quat": [0.0, 0.0, 0.0, 1.0],
            },
        }
        os.makedirs(os.path.dirname(os.path.abspath(self._output_file)), exist_ok=True)
        with open(self._output_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(f"Saved VR calibration to {self._output_file}")


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
