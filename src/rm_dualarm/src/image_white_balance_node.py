#!/usr/bin/python3
"""Apply precomputed RGB white-balance gains to a sensor_msgs/Image stream."""

from typing import Dict, Optional

import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


SUPPORTED_ENCODINGS = {
    "rgb8": np.array([0, 1, 2], dtype=np.int64),
    "bgr8": np.array([2, 1, 0], dtype=np.int64),
}


class ImageWhiteBalance(Node):
    def __init__(self):
        super().__init__("image_white_balance")

        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/camera/image_wb")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("camera_key", "")
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("input_reliability", "best_effort")
        self.declare_parameter("output_reliability", "reliable")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._calibration_file = str(self.get_parameter("calibration_file").value)
        self._camera_key = str(self.get_parameter("camera_key").value)
        self._queue_size = int(self.get_parameter("queue_size").value)
        self._input_reliability = str(self.get_parameter("input_reliability").value)
        self._output_reliability = str(self.get_parameter("output_reliability").value)
        if self._queue_size <= 0:
            raise ValueError("queue_size must be positive")
        if not self._calibration_file:
            raise ValueError("calibration_file parameter is required")

        self._gains_rgb = self._load_gains()
        self._warned_encodings = set()

        input_qos = self._make_qos(self._input_reliability)
        output_qos = self._make_qos(self._output_reliability)

        self._publisher = self.create_publisher(
            Image,
            self._output_topic,
            output_qos,
        )
        self.create_subscription(
            Image,
            self._input_topic,
            self._image_cb,
            input_qos,
        )

        self.get_logger().info(
            f"Applying white balance {self._gains_rgb.tolist()} from "
            f"{self._input_topic} to {self._output_topic} "
            f"(input_reliability={self._input_reliability}, "
            f"output_reliability={self._output_reliability})"
        )

    def _make_qos(self, reliability: str) -> QoSProfile:
        reliability = reliability.lower()
        if reliability in ("best_effort", "best-effort", "besteffort"):
            reliability_policy = ReliabilityPolicy.BEST_EFFORT
        elif reliability == "reliable":
            reliability_policy = ReliabilityPolicy.RELIABLE
        else:
            raise ValueError(
                "reliability must be 'best_effort' or 'reliable', "
                f"got '{reliability}'"
            )

        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self._queue_size,
            reliability=reliability_policy,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _load_gains(self) -> np.ndarray:
        with open(self._calibration_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        root = data.get("white_balance", data)
        cameras: Dict = root.get("cameras", {})
        if not cameras:
            raise ValueError(f"{self._calibration_file} does not contain white_balance.cameras")

        camera = None
        if self._camera_key:
            camera = cameras.get(self._camera_key)
            if camera is None:
                raise ValueError(
                    f"camera_key '{self._camera_key}' not found in {self._calibration_file}"
                )
        else:
            for candidate in cameras.values():
                if candidate.get("topic") == self._input_topic:
                    camera = candidate
                    break
            if camera is None and len(cameras) == 1:
                camera = next(iter(cameras.values()))
            if camera is None:
                raise ValueError(
                    "camera_key is required when calibration_file contains multiple cameras "
                    "and no topic matches input_topic"
                )

        gains = camera.get("gains_rgb")
        if gains is None:
            gains = camera.get("gains")
        if not isinstance(gains, list) or len(gains) != 3:
            raise ValueError("white-balance gains must be a 3-element list")

        gains_rgb = np.asarray(gains, dtype=np.float32)
        if not np.all(np.isfinite(gains_rgb)) or np.any(gains_rgb <= 0.0):
            raise ValueError(f"invalid gains_rgb: {gains}")
        return gains_rgb

    def _image_cb(self, msg: Image):
        encoding = msg.encoding.lower()
        if encoding not in SUPPORTED_ENCODINGS:
            if encoding not in self._warned_encodings:
                self.get_logger().error(
                    f"Unsupported image encoding '{msg.encoding}'. "
                    f"Supported: {', '.join(sorted(SUPPORTED_ENCODINGS))}"
                )
                self._warned_encodings.add(encoding)
            return

        channels = 3
        expected_step = msg.width * channels
        if msg.step < expected_step:
            self.get_logger().error(
                f"Invalid image step {msg.step}; expected at least {expected_step}"
            )
            return

        expected_len = msg.height * msg.step
        source = np.frombuffer(msg.data, dtype=np.uint8)
        if source.size < expected_len:
            self.get_logger().error(
                f"Truncated image data: {source.size} bytes, expected {expected_len}"
            )
            return

        output = source[:expected_len].copy()
        rows = output.reshape(msg.height, msg.step)
        pixels = rows[:, :expected_step].reshape(msg.height, msg.width, channels)
        channel_order = SUPPORTED_ENCODINGS[encoding]
        gains = self._gains_rgb[channel_order]
        corrected = np.clip(pixels.astype(np.float32) * gains, 0.0, 255.0).astype(np.uint8)
        pixels[:, :, :] = corrected

        out_msg = Image()
        out_msg.header = msg.header
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.encoding = msg.encoding
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.step = msg.step
        out_msg.data = output.tobytes()
        self._publisher.publish(out_msg)


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageWhiteBalance()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
