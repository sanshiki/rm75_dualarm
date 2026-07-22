#!/usr/bin/python3
"""Estimate per-camera RGB white-balance gains from a rosbag2 image ROI."""

import argparse
from datetime import datetime, timezone
import os
import re
from typing import Dict, Iterable, List, Tuple

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_TOPICS = [
    "/camera/image_raw",
    "/wrist_camera/color/image_raw",
]

SUPPORTED_ENCODINGS = {
    "rgb8": (3, (0, 1, 2)),
    "bgr8": (3, (2, 1, 0)),
}


def _split_csv(values: Iterable[str]) -> List[str]:
    result = []
    for value in values:
        for item in value.split(","):
            item = item.strip()
            if item:
                result.append(item)
    return result


def _default_camera_key(topic: str) -> str:
    key = topic.strip("/")
    key = re.sub(r"/color/image_raw$", "", key)
    key = re.sub(r"/image_raw$", "", key)
    key = re.sub(r"[^A-Za-z0-9_]+", "_", key)
    return key or "camera"


def _parse_roi(spec: str, width: int, height: int) -> Tuple[int, int, int, int, Dict]:
    spec = spec.strip().lower()
    parts = spec.split(":")
    if parts[0] == "center":
        center_x = center_y = 0.5
        if len(parts) == 1:
            frac_w = frac_h = 0.4
        elif len(parts) == 2:
            frac_w = frac_h = float(parts[1])
        elif len(parts) == 3:
            frac_w = float(parts[1])
            frac_h = float(parts[2])
        elif len(parts) == 5:
            center_x = float(parts[1])
            center_y = float(parts[2])
            frac_w = float(parts[3])
            frac_h = float(parts[4])
        else:
            raise ValueError(
                "center ROI must be center[:width_fraction[:height_fraction]] "
                "or center:center_x:center_y:width_fraction:height_fraction"
            )
        if not (
            0.0 <= center_x <= 1.0
            and 0.0 <= center_y <= 1.0
            and 0.0 < frac_w <= 1.0
            and 0.0 < frac_h <= 1.0
        ):
            raise ValueError("center ROI values must be normalized to [0, 1]")
        roi_w = max(1, int(round(width * frac_w)))
        roi_h = max(1, int(round(height * frac_h)))
        center_px = int(round(width * center_x))
        center_py = int(round(height * center_y))
        x = center_px - roi_w // 2
        y = center_py - roi_h // 2
        x, y, roi_w, roi_h = _clamp_roi(x, y, roi_w, roi_h, width, height)
        return x, y, roi_w, roi_h, {
            "mode": "center",
            "center_x_fraction": center_x,
            "center_y_fraction": center_y,
            "width_fraction": frac_w,
            "height_fraction": frac_h,
        }

    if parts[0] in ("fraction", "frac"):
        if len(parts) != 5:
            raise ValueError("fraction ROI must be fraction:x:y:width:height")
        x_f, y_f, w_f, h_f = (float(v) for v in parts[1:])
        if not (0.0 <= x_f < 1.0 and 0.0 <= y_f < 1.0 and 0.0 < w_f <= 1.0 and 0.0 < h_f <= 1.0):
            raise ValueError("fraction ROI values must be normalized to [0, 1]")
        x = int(round(width * x_f))
        y = int(round(height * y_f))
        roi_w = int(round(width * w_f))
        roi_h = int(round(height * h_f))
        x, y, roi_w, roi_h = _clamp_roi(x, y, roi_w, roi_h, width, height)
        return x, y, roi_w, roi_h, {
            "mode": "fraction",
            "x": x_f,
            "y": y_f,
            "width": w_f,
            "height": h_f,
        }

    if parts[0] in ("pixel", "px"):
        if len(parts) != 5:
            raise ValueError("pixel ROI must be pixel:x:y:width:height")
        x, y, roi_w, roi_h = (int(v) for v in parts[1:])
        x, y, roi_w, roi_h = _clamp_roi(x, y, roi_w, roi_h, width, height)
        return x, y, roi_w, roi_h, {
            "mode": "pixel",
            "x": x,
            "y": y,
            "width": roi_w,
            "height": roi_h,
        }

    raise ValueError("ROI must start with center, fraction, or pixel")


def _clamp_roi(
    x: int, y: int, roi_w: int, roi_h: int, width: int, height: int
) -> Tuple[int, int, int, int]:
    if roi_w <= 0 or roi_h <= 0:
        raise ValueError("ROI width and height must be positive")
    x = min(max(0, x), width - 1)
    y = min(max(0, y), height - 1)
    roi_w = min(roi_w, width - x)
    roi_h = min(roi_h, height - y)
    return x, y, roi_w, roi_h


class TopicStats:
    def __init__(self, topic: str, camera_key: str):
        self.topic = topic
        self.camera_key = camera_key
        self.encoding = ""
        self.width = 0
        self.height = 0
        self.frames_seen = 0
        self.frames_used = 0
        self.valid_pixels = 0
        self.rgb_sum = np.zeros(3, dtype=np.float64)
        self.roi_pixels = None
        self.roi_meta = None

    def add_image(self, msg, roi_spec: str, min_value: int, max_value: int, min_pixels: int):
        self.frames_seen += 1
        encoding = msg.encoding.lower()
        if encoding not in SUPPORTED_ENCODINGS:
            raise ValueError(
                f"{self.topic} uses unsupported encoding '{msg.encoding}'. "
                f"Supported: {', '.join(sorted(SUPPORTED_ENCODINGS))}"
            )

        channels, rgb_order = SUPPORTED_ENCODINGS[encoding]
        expected_step = msg.width * channels
        if msg.step < expected_step:
            raise ValueError(
                f"{self.topic} has invalid step {msg.step}; expected at least {expected_step}"
            )

        data = np.frombuffer(msg.data, dtype=np.uint8)
        expected_len = msg.height * msg.step
        if data.size < expected_len:
            raise ValueError(
                f"{self.topic} data is truncated: {data.size} bytes, expected {expected_len}"
            )

        rows = data[:expected_len].reshape(msg.height, msg.step)
        image = rows[:, :expected_step].reshape(msg.height, msg.width, channels)
        rgb_image = image[:, :, rgb_order]

        if self.roi_pixels is None:
            self.width = int(msg.width)
            self.height = int(msg.height)
            self.encoding = encoding
            x, y, roi_w, roi_h, roi_meta = _parse_roi(roi_spec, self.width, self.height)
            self.roi_pixels = (x, y, roi_w, roi_h)
            self.roi_meta = roi_meta

        x, y, roi_w, roi_h = self.roi_pixels
        roi = rgb_image[y:y + roi_h, x:x + roi_w, :].astype(np.float32)
        mask = (roi.min(axis=2) >= min_value) & (roi.max(axis=2) <= max_value)
        count = int(mask.sum())
        if count < min_pixels:
            return

        self.frames_used += 1
        self.valid_pixels += count
        self.rgb_sum += roi[mask].sum(axis=0)

    def result(self) -> Dict:
        if self.valid_pixels <= 0:
            raise RuntimeError(f"{self.topic}: no valid ROI pixels after filtering")
        mean = self.rgb_sum / float(self.valid_pixels)
        gray = float(mean.mean())
        gains = gray / np.maximum(mean, 1.0)
        x, y, roi_w, roi_h = self.roi_pixels
        return {
            "topic": self.topic,
            "encoding": self.encoding,
            "image_size": [self.width, self.height],
            "roi_pixels": {
                "x": int(x),
                "y": int(y),
                "width": int(roi_w),
                "height": int(roi_h),
            },
            "roi_rgb_mean": [round(float(v), 4) for v in mean],
            "gains_rgb": [round(float(v), 6) for v in gains],
            "frames_seen": int(self.frames_seen),
            "frames_used": int(self.frames_used),
            "valid_pixels": int(self.valid_pixels),
        }


def calibrate(args) -> Dict:
    topics = _split_csv(args.topics)
    if not topics:
        raise ValueError("At least one image topic is required")

    keys = _split_csv(args.camera_keys or [])
    if keys and len(keys) != len(topics):
        raise ValueError("--camera-keys must have the same count as --topics")
    if not keys:
        keys = [_default_camera_key(topic) for topic in topics]
    if len(set(keys)) != len(keys):
        raise ValueError(f"camera keys must be unique, got: {keys}")

    stats = {
        topic: TopicStats(topic, camera_key)
        for topic, camera_key in zip(topics, keys)
    }

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id=args.storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics if topic not in topic_types]
    if missing:
        raise RuntimeError(f"Topics not found in bag: {', '.join(missing)}")

    image_types = {
        topic: get_message(topic_types[topic])
        for topic in topics
    }
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in stats:
            continue
        msg = deserialize_message(data, image_types[topic])
        stats[topic].add_image(
            msg,
            args.roi,
            args.min_value,
            args.max_value,
            args.min_pixels,
        )

    cameras = {}
    for topic in topics:
        result = stats[topic].result()
        cameras[stats[topic].camera_key] = result

    first_stats = next(iter(stats.values()))
    return {
        "white_balance": {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "generated_from_bag": args.bag,
            "method": "white_paper_roi_channel_gain",
            "roi": first_stats.roi_meta or {"mode": "unknown"},
            "value_filter": {
                "min_value": int(args.min_value),
                "max_value": int(args.max_value),
                "min_pixels_per_frame": int(args.min_pixels),
            },
            "cameras": cameras,
        }
    }


def main():
    parser = argparse.ArgumentParser(
        description="Estimate RGB white-balance gains from white-paper ROIs in a rosbag2."
    )
    parser.add_argument("--bag", required=True, help="Path to a rosbag2 directory")
    parser.add_argument(
        "--topics",
        nargs="+",
        default=DEFAULT_TOPICS,
        help="Image topics to calibrate. Comma-separated values are also accepted.",
    )
    parser.add_argument(
        "--camera-keys",
        nargs="+",
        default=None,
        help="Optional YAML keys corresponding to --topics.",
    )
    parser.add_argument(
        "--output",
        default="src/rm_dualarm/config/white_balance.yaml",
        help="Output YAML path.",
    )
    parser.add_argument(
        "--roi",
        default="center:0.5:0.5:0.2:0.2",
        help=(
            "ROI spec: center[:wf[:hf]], center:cx:cy:wf:hf, "
            "fraction:x:y:w:h, or pixel:x:y:w:h."
        ),
    )
    parser.add_argument("--min-value", type=int, default=20, help="Reject darker ROI pixels.")
    parser.add_argument("--max-value", type=int, default=245, help="Reject saturated ROI pixels.")
    parser.add_argument(
        "--min-pixels",
        type=int,
        default=100,
        help="Minimum valid ROI pixels required for a frame to contribute.",
    )
    parser.add_argument("--storage-id", default="sqlite3", help="rosbag2 storage id.")
    args = parser.parse_args()

    if not (0 <= args.min_value <= 255 and 0 <= args.max_value <= 255):
        raise ValueError("--min-value and --max-value must be in [0, 255]")
    if args.min_value >= args.max_value:
        raise ValueError("--min-value must be less than --max-value")

    data = calibrate(args)
    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    for key, camera in data["white_balance"]["cameras"].items():
        gains = ", ".join(f"{value:.6f}" for value in camera["gains_rgb"])
        mean = ", ".join(f"{value:.2f}" for value in camera["roi_rgb_mean"])
        print(
            f"{key}: topic={camera['topic']} frames={camera['frames_used']}/"
            f"{camera['frames_seen']} mean_rgb=[{mean}] gains_rgb=[{gains}]"
        )
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
