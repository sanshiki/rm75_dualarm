#!/usr/bin/env python3
"""Convert ROS 2 rosbag episodes to raw episode.npy format.

Default input is bags/single and default output is convert_result.
"""

import argparse
import json
import shutil
import subprocess
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R


DEFAULT_BAG_ROOT = "bags/single"
DEFAULT_OUTPUT_ROOT = "convert_result"
DEFAULT_WHITE_BALANCE_CONFIG = "src/rm_dualarm/config/white_balance.yaml"

FIXED_TOPICS = {
    "camera": "/camera/image_raw",
    "joint_states": "/joint_states",
    "tf": "/tf",
    "tf_static": "/tf_static",
}

WRIST_CAMERA_CANDIDATES = [
    "/wrist_camera/color/image_raw",
    "/left/wrist_camera/color/image_raw",
    "/right/wrist_camera/color/image_raw",
]
TARGET_CANDIDATES = ["/target_pose"]
GRIPPER_CANDIDATES = [
    "/rm_driver/set_gripper_position_cmd",
    "/gripper_cmd",
]
TRAJ_CMD_CANDIDATES = [
    "/rm_group_controller/joint_trajectory",
    "/servo_bridge/joint_trajectory_in",
    "/rm_driver/movej_canfd_cmd",
]

BASE_FRAME = "base_link"
LINK_CHAIN = [f"Link{i}" for i in range(1, 8)]
ACTION_LAYOUT = ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"]


def slugify(text):
    chars = []
    previous_underscore = False
    for ch in text.strip().lower():
        if ch.isalnum():
            chars.append(ch)
            previous_underscore = False
        elif not previous_underscore:
            chars.append("_")
            previous_underscore = True
    slug = "".join(chars).strip("_")
    return slug or "teleop_task"


def discover_bags(root):
    root = Path(root)
    if root.is_file():
        raise ValueError(f"bag path must be a rosbag directory or parent directory: {root}")
    if (root / "metadata.yaml").exists():
        return [root]
    bags = [p for p in root.rglob("metadata.yaml") if p.parent.is_dir()]
    return sorted({p.parent for p in bags})


def resolve_topics(topic_types):
    topics = set(topic_types)
    resolved = {}
    for key, topic in FIXED_TOPICS.items():
        if topic in topics:
            resolved[key] = topic
    for topic in WRIST_CAMERA_CANDIDATES:
        if topic in topics:
            resolved["wrist_camera"] = topic
            break
    for topic in TARGET_CANDIDATES:
        if topic in topics:
            resolved["target_pose"] = topic
            break
    for topic in GRIPPER_CANDIDATES:
        if topic in topics:
            resolved["gripper_cmd"] = topic
            break
    for topic in TRAJ_CMD_CANDIDATES:
        if topic in topics:
            resolved["traj_cmd"] = topic
            break
    return resolved


def read_bag(path, wanted_topics):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    messages = defaultdict(list)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in wanted_topics:
            continue
        msg_type = get_message(topic_types[topic])
        messages[topic].append((stamp * 1e-9, deserialize_message(data, msg_type)))
    return topic_types, messages


def get_topic_types(path):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def ros_time_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def msg_time(default_time, msg):
    header = getattr(msg, "header", None)
    if header is not None:
        stamp = getattr(header, "stamp", None)
        if stamp is not None and (stamp.sec or stamp.nanosec):
            return ros_time_to_float(stamp)
    return default_time


def image_to_rgb(msg):
    height = int(msg.height)
    width = int(msg.width)
    encoding = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in ("rgb8", "bgr8"):
        image = data.reshape(height, int(msg.step))[:, : width * 3].reshape(height, width, 3)
        if encoding == "bgr8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    if encoding in ("rgba8", "bgra8"):
        image = data.reshape(height, int(msg.step))[:, : width * 4].reshape(height, width, 4)
        if encoding == "bgra8":
            image = image[:, :, [2, 1, 0, 3]]
        return np.ascontiguousarray(image[:, :, :3])

    if encoding in ("mono8", "8uc1"):
        image = data.reshape(height, int(msg.step))[:, :width]
        return np.ascontiguousarray(np.repeat(image[:, :, None], 3, axis=2))

    if encoding in ("8uc3",):
        image = data.reshape(height, int(msg.step))[:, : width * 3].reshape(height, width, 3)
        return np.ascontiguousarray(image)

    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def apply_white_balance(image, gains_rgb):
    if gains_rgb is None:
        return image
    gains = np.asarray(gains_rgb, dtype=np.float32)
    return np.clip(image.astype(np.float32) * gains, 0.0, 255.0).astype(np.uint8)


def load_white_balance_config(path):
    if not path:
        return {}
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"white balance config not found: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    root = data.get("white_balance", data)
    cameras = root.get("cameras", {})
    if not isinstance(cameras, dict):
        raise ValueError(f"{config_path} does not contain white_balance.cameras")
    return cameras


def _camera_gains_from_config(cameras, camera_key, topic):
    camera = cameras.get(camera_key)
    if camera is None:
        for candidate in cameras.values():
            if candidate.get("topic") == topic:
                camera = candidate
                break
    if camera is None:
        return None

    gains = camera.get("gains_rgb", camera.get("gains"))
    if not isinstance(gains, list) or len(gains) != 3:
        raise ValueError(f"white balance gains for {camera_key} must be a 3-element list")
    gains_rgb = np.asarray(gains, dtype=np.float32)
    if not np.all(np.isfinite(gains_rgb)) or np.any(gains_rgb <= 0.0):
        raise ValueError(f"invalid white balance gains for {camera_key}: {gains}")
    return gains_rgb


def resolve_white_balance_gains(resolved, args):
    if not args.apply_white_balance:
        return {}, None
    cameras = load_white_balance_config(args.white_balance_config)
    gains = {}
    for key in ("camera", "wrist_camera"):
        topic = resolved.get(key)
        if not topic:
            continue
        camera_gains = _camera_gains_from_config(cameras, key, topic)
        if camera_gains is not None:
            gains[key] = camera_gains
        else:
            print(f"[warn] no white balance gains for {key} topic {topic}")
    return gains, str(Path(args.white_balance_config))


def resolve_action_normalization(args):
    if args.action_min is None and args.action_max is None:
        return None
    if args.action_min is None or args.action_max is None:
        raise ValueError("--action-min and --action-max must be provided together")

    raw_min = np.asarray(args.action_min, dtype=np.float32)
    raw_max = np.asarray(args.action_max, dtype=np.float32)
    target_min, target_max = (float(value) for value in args.action_normalized_range)

    if raw_min.shape != (len(ACTION_LAYOUT),) or raw_max.shape != (len(ACTION_LAYOUT),):
        raise ValueError(f"--action-min and --action-max must each have {len(ACTION_LAYOUT)} values")
    if not np.all(np.isfinite(raw_min)) or not np.all(np.isfinite(raw_max)):
        raise ValueError("--action-min and --action-max must be finite")
    if np.any(raw_max <= raw_min):
        raise ValueError("--action-max values must be greater than --action-min values")
    if not np.isfinite(target_min) or not np.isfinite(target_max) or target_max <= target_min:
        raise ValueError("--action-normalized-range must be finite and increasing")

    return {
        "raw_min": raw_min,
        "raw_max": raw_max,
        "target_min": target_min,
        "target_max": target_max,
        "clip": bool(args.clip_normalized_action),
    }


def normalize_action(action, normalization):
    action = np.asarray(action, dtype=np.float32)
    if normalization is None:
        return action
    raw_min = normalization["raw_min"]
    raw_max = normalization["raw_max"]
    target_min = normalization["target_min"]
    target_max = normalization["target_max"]
    normalized = (action - raw_min) / (raw_max - raw_min)
    normalized = normalized * (target_max - target_min) + target_min
    # handle gripper value
    normalized[6] = -1 if normalized[6] < 0 else 1
    if normalization["clip"]:
        normalized = np.clip(normalized, target_min, target_max)
    return normalized.astype(np.float32)


def action_normalization_metadata(normalization):
    if normalization is None:
        return {
            "normalized": False,
            "layout": ACTION_LAYOUT,
        }
    return {
        "normalized": True,
        "layout": ACTION_LAYOUT,
        "raw_min": [float(value) for value in normalization["raw_min"]],
        "raw_max": [float(value) for value in normalization["raw_max"]],
        "target_min": float(normalization["target_min"]),
        "target_max": float(normalization["target_max"]),
        "clip": bool(normalization["clip"]),
    }


def pose_record(time_sec, pose):
    q = pose.orientation
    p = pose.position
    return {
        "time": time_sec,
        "position": np.array([p.x, p.y, p.z], dtype=np.float32),
        "quat": np.array([q.x, q.y, q.z, q.w], dtype=np.float32),
    }


def target_pose_records(rows):
    records = []
    for t, msg in rows:
        records.append(pose_record(msg_time(t, msg), msg.pose))
    return sorted(records, key=lambda item: item["time"])


def transform_matrix(trans):
    p = trans.transform.translation
    q = trans.transform.rotation
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[:3, 3] = [p.x, p.y, p.z]
    return mat


def matrix_pose_record(time_sec, mat):
    rot = R.from_matrix(mat[:3, :3])
    return {
        "time": time_sec,
        "position": mat[:3, 3].astype(np.float32),
        "quat": rot.as_quat().astype(np.float32),
    }


def eef_pose_records(tf_rows, tf_static_rows):
    ee_frame = LINK_CHAIN[-1]
    records = []
    static_edges = {}
    for _, msg in tf_static_rows:
        for trans in msg.transforms:
            static_edges[(trans.header.frame_id, trans.child_frame_id)] = transform_matrix(trans)

    latest_edges = dict(static_edges)
    chain_edges = []
    parent = BASE_FRAME
    for child in LINK_CHAIN:
        chain_edges.append((parent, child))
        parent = child

    for t, msg in tf_rows:
        time_sec = msg_time(t, msg.transforms[0]) if msg.transforms else t
        appended_direct = False
        for trans in msg.transforms:
            latest_edges[(trans.header.frame_id, trans.child_frame_id)] = transform_matrix(trans)
            if trans.header.frame_id == BASE_FRAME and trans.child_frame_id == ee_frame:
                records.append(matrix_pose_record(msg_time(t, trans), latest_edges[(BASE_FRAME, ee_frame)]))
                appended_direct = True

        if not appended_direct and all(edge in latest_edges for edge in chain_edges):
            mat = np.eye(4, dtype=np.float64)
            for edge in chain_edges:
                mat = mat @ latest_edges[edge]
            records.append(matrix_pose_record(time_sec, mat))
    return sorted(records, key=lambda item: item["time"])


def joint_state_records(rows):
    records = []
    for t, msg in rows:
        records.append({
            "time": msg_time(t, msg),
            "name": list(msg.name),
            "position": np.asarray(msg.position, dtype=np.float32),
        })
    return sorted(records, key=lambda item: item["time"])


def traj_cmd_records(rows):
    records = []
    for t, msg in rows:
        if hasattr(msg, "joint") and msg.joint:
            records.append({
                "time": t,
                "position": np.asarray(msg.joint, dtype=np.float32),
            })
        elif hasattr(msg, "points") and msg.points:
            records.append({
                "time": msg_time(t, msg),
                "joint_names": list(msg.joint_names),
                "position": np.asarray(msg.points[0].positions, dtype=np.float32),
            })
    return sorted(records, key=lambda item: item["time"])


def gripper_records(rows):
    records = []
    for t, msg in rows:
        if hasattr(msg, "position"):
            value = float(msg.position) / 1000.0
        elif hasattr(msg, "data"):
            value = float(msg.data)
        else:
            continue
        records.append({"time": msg_time(t, msg), "value": float(np.clip(value, 0.0, 1.0))})
    return sorted(records, key=lambda item: item["time"])


def image_records(rows, gains_rgb=None):
    records = []
    for t, msg in rows:
        image = apply_white_balance(image_to_rgb(msg), gains_rgb)
        records.append({"time": msg_time(t, msg), "image": image})
    return sorted(records, key=lambda item: item["time"])


def nearest(records, time_sec, max_dt=None):
    if not records:
        return None
    times = np.asarray([item["time"] for item in records], dtype=np.float64)
    idx = int(np.argmin(np.abs(times - time_sec)))
    if max_dt is not None and abs(times[idx] - time_sec) > max_dt:
        return None
    return records[idx]


def latest_at_or_before(records, times, time_sec):
    if not records:
        return None
    idx = int(np.searchsorted(times, time_sec, side="right")) - 1
    if idx < 0:
        return None
    return records[idx]


def pose_delta(current, target):
    dp = np.asarray(target["position"], dtype=np.float64) - np.asarray(current["position"], dtype=np.float64)
    current_rot = R.from_quat(current["quat"])
    target_rot = R.from_quat(target["quat"])
    drot = (target_rot * current_rot.inv()).as_euler("xyz", degrees=False)
    return np.concatenate([dp, drot]).astype(np.float32)


def sample_times(image_recs, eef_recs, target_recs, control_hz):
    if not image_recs or not eef_recs:
        return []
    start = max(image_recs[0]["time"], eef_recs[0]["time"])
    end = min(image_recs[-1]["time"], eef_recs[-1]["time"])
    if target_recs:
        start = max(start, target_recs[0]["time"])
        end = min(end, target_recs[-1]["time"])
    if end <= start:
        return []
    step = 1.0 / float(control_hz)
    return list(np.arange(start, end + 1e-9, step))


def set_step_gripper(step, gripper_value, action_normalization):
    gripper_value = float(gripper_value)
    step["gripper_cmd"] = gripper_value
    if "raw_action" in step:
        raw_action = np.asarray(step["raw_action"], dtype=np.float32)
        raw_action[6] = gripper_value
        step["raw_action"] = raw_action.astype(np.float32).tolist()
        step["action"] = normalize_action(raw_action, action_normalization).tolist()
        return

    action = np.asarray(step["action"], dtype=np.float32)
    action[6] = gripper_value
    step["action"] = action.astype(np.float32).tolist()


def apply_gripper_event_overrides(episode, grippers, action_normalization, max_dt):
    if not episode or not grippers or max_dt <= 0.0:
        return 0

    step_times = np.asarray([step["timestamp"] for step in episode], dtype=np.float64)
    updated = 0
    for gripper in grippers:
        event_time = float(gripper["time"])
        idx = int(np.searchsorted(step_times, event_time, side="left"))
        selected = None

        if idx < len(step_times) and (step_times[idx] - event_time) <= max_dt:
            selected = idx
        elif idx > 0 and (event_time - step_times[idx - 1]) <= max_dt:
            selected = idx - 1

        if selected is None:
            continue

        before = float(episode[selected].get("gripper_cmd", np.nan))
        set_step_gripper(episode[selected], gripper["value"], action_normalization)
        if not np.isfinite(before) or abs(before - float(gripper["value"])) > 1e-6:
            updated += 1

    return updated


def make_episode(messages, resolved, args, white_balance_gains, action_normalization):
    images = image_records(
        messages.get(resolved.get("camera", ""), []),
        white_balance_gains.get("camera"),
    )
    wrist_images = image_records(
        messages.get(resolved.get("wrist_camera", ""), []),
        white_balance_gains.get("wrist_camera"),
    )
    eef = eef_pose_records(
        messages.get(resolved.get("tf", ""), []),
        messages.get(resolved.get("tf_static", ""), []),
    )
    targets = target_pose_records(messages.get(resolved.get("target_pose", ""), []))
    joints = joint_state_records(messages.get(resolved.get("joint_states", ""), []))
    traj_cmds = traj_cmd_records(messages.get(resolved.get("traj_cmd", ""), []))
    grippers = gripper_records(messages.get(resolved.get("gripper_cmd", ""), []))
    gripper_times = np.asarray([item["time"] for item in grippers], dtype=np.float64)

    if not images:
        raise RuntimeError("missing camera images")
    if not eef:
        raise RuntimeError("missing end-effector TF")

    use_target_pose = bool(targets)
    use_wrist_camera = bool(wrist_images)
    times = sample_times(images, eef, targets if use_target_pose else [], args.control_hz)
    episode = []
    max_image_dt = 0.5 / float(args.control_hz)
    max_state_dt = 1.0 / float(args.control_hz)

    for index, time_sec in enumerate(times):
        image = nearest(images, time_sec, max_dt=max_image_dt)
        wrist_image = nearest(wrist_images, time_sec, max_dt=max_image_dt) if use_wrist_camera else None
        current = nearest(eef, time_sec, max_dt=max_state_dt)
        if image is None or current is None:
            continue
        if use_wrist_camera and wrist_image is None:
            continue

        target = nearest(targets, time_sec, max_dt=max_state_dt) if use_target_pose else None
        if target is None:
            if index + 1 >= len(times):
                continue
            target = nearest(eef, times[index + 1], max_dt=max_state_dt)
        if target is None:
            continue

        gripper = latest_at_or_before(grippers, gripper_times, time_sec)
        gripper_value = float(gripper["value"]) if gripper else args.default_gripper
        raw_action = np.concatenate([pose_delta(current, target), np.array([gripper_value], dtype=np.float32)])
        action = normalize_action(raw_action, action_normalization)

        joint = nearest(joints, time_sec)
        traj_cmd = nearest(traj_cmds, time_sec)
        step = {
            "images": image["image"],
            "task_description": args.task_description,
            "action": action.astype(np.float32).tolist(),
            "timestamp": float(time_sec),
            "image_timestamp": float(image["time"]),
            "control_timestamp": float(time_sec),
            "ee_pose": np.concatenate([current["position"], current["quat"]]).astype(np.float32),
            "target_pose": np.concatenate([target["position"], target["quat"]]).astype(np.float32),
            "gripper_cmd": float(gripper_value),
        }
        if action_normalization is not None:
            step["raw_action"] = raw_action.astype(np.float32).tolist()
        if wrist_image is not None:
            step["wrist_images"] = wrist_image["image"]
            step["wrist_image_timestamp"] = float(wrist_image["time"])
        if joint is not None:
            step["joint_state"] = joint["position"]
            step["joint_names"] = joint["name"]
        if traj_cmd is not None:
            step["traj_cmd"] = traj_cmd["position"]
        episode.append(step)

    if not episode:
        raise RuntimeError("no synchronized timesteps produced")

    gripper_event_overrides = apply_gripper_event_overrides(
        episode,
        grippers,
        action_normalization,
        args.gripper_event_max_dt,
    )
    duration = episode[-1]["timestamp"] - episode[0]["timestamp"] if len(episode) > 1 else 0.0
    image_shape = list(episode[0]["images"].shape)
    wrist_image_shape = list(episode[0]["wrist_images"].shape) if "wrist_images" in episode[0] else None
    sync_policy = "control_time_nearest_neighbor"
    action_source = "target_pose_delta" if use_target_pose else "eef_next_pose_delta"
    return episode, {
        "duration_sec": float(duration),
        "image_shape": image_shape,
        "wrist_image_shape": wrist_image_shape,
        "sync_policy": sync_policy,
        "action_source": action_source,
        "gripper_sync_policy": "zero_order_hold_at_or_before_with_event_frame",
        "gripper_command_count": len(grippers),
        "gripper_event_override_count": gripper_event_overrides,
    }


def write_video_ffmpeg(path, frames, fps, ffmpeg_bin):
    if not frames:
        return "none"
    first = frames[0]
    height, width = first.shape[:2]
    cmd = [
        ffmpeg_bin,
        "-y",
        "-f", "rawvideo",
        "-vcodec", "rawvideo",
        "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}",
        "-r", str(float(fps)),
        "-i", "-",
        "-an",
        "-vcodec", "libx264",
        "-preset", "veryfast",
        "-crf", "23",
        "-pix_fmt", "yuv420p",
        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        "-movflags", "+faststart",
        str(path),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for rgb in frames:
            if rgb.shape[:2] != (height, width):
                raise RuntimeError("all video frames must have the same resolution")
            proc.stdin.write(np.ascontiguousarray(rgb, dtype=np.uint8).tobytes())
        proc.stdin.close()
        stderr = proc.stderr.read().decode("utf-8", errors="replace")
        return_code = proc.wait()
    except Exception:
        proc.kill()
        proc.wait()
        raise
    if return_code != 0:
        raise RuntimeError(f"ffmpeg failed while writing {path}:\n{stderr[-2000:]}")
    return "h264"


def write_video_opencv(path, frames, fps):
    if not frames:
        return "none"
    first = frames[0]
    height, width = first.shape[:2]
    codecs = ("avc1", "H264", "mp4v")
    writer = None
    codec = None
    for fourcc_text in codecs:
        fourcc = cv2.VideoWriter_fourcc(*fourcc_text)
        candidate = cv2.VideoWriter(str(path), fourcc, float(fps), (width, height))
        if candidate.isOpened():
            writer = candidate
            codec = fourcc_text.lower()
            break
        candidate.release()
    if writer is None:
        raise RuntimeError(f"failed to open video writer: {path}")
    try:
        for rgb in frames:
            if rgb.shape[:2] != (height, width):
                raise RuntimeError("all video frames must have the same resolution")
            writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    finally:
        writer.release()
    return codec


def write_video(path, frames, fps):
    ffmpeg_bin = shutil.which("ffmpeg")
    if ffmpeg_bin:
        return write_video_ffmpeg(path, frames, fps, ffmpeg_bin)
    print("[warn] ffmpeg not found; falling back to OpenCV video writer")
    return write_video_opencv(path, frames, fps)


def write_episode(out_dir, episode, metadata, write_mp4, fps):
    out_dir.mkdir(parents=True, exist_ok=True)
    np.save(out_dir / "episode.npy", np.asarray(episode, dtype=object), allow_pickle=True)
    if write_mp4:
        codec = write_video(out_dir / "episode.mp4", [step["images"] for step in episode], fps)
        metadata["image"]["video_codec"] = codec
        if "wrist_images" in episode[0]:
            wrist_codec = write_video(
                out_dir / "wrist_episode.mp4",
                [step["wrist_images"] for step in episode],
                fps,
            )
            metadata["image"]["wrist_video_codec"] = wrist_codec
    with open(out_dir / "metadata.json", "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)


def convert_bag(bag_path, out_dir, episode_id, resolved, messages, args, stats):
    white_balance_gains, white_balance_config = resolve_white_balance_gains(resolved, args)
    episode, extra = make_episode(
        messages,
        resolved,
        args,
        white_balance_gains,
        args.action_normalization,
    )
    white_balance_metadata = {
        key: [float(value) for value in gains]
        for key, gains in white_balance_gains.items()
    }
    metadata = {
        "task_description": args.task_description,
        "episode": {
            "id": episode_id,
            "file": "episode.npy",
            "video": "episode.mp4" if args.video else None,
            "wrist_video": (
                "wrist_episode.mp4"
                if args.video and extra["wrist_image_shape"] is not None
                else None
            ),
            "success": args.success,
            "score": args.score,
            "num_steps": len(episode),
            "duration_sec": extra["duration_sec"],
        },
        "source": {
            "type": "ros2_rosbag",
            "rosbag": str(bag_path.resolve()),
            "camera_topic": resolved.get("camera"),
            "wrist_camera_topic": resolved.get("wrist_camera"),
            "tf_topic": resolved.get("tf"),
            "target_pose_topic": resolved.get("target_pose"),
            "traj_cmd_topic": resolved.get("traj_cmd"),
            "joint_states_topic": resolved.get("joint_states"),
            "gripper_cmd_topic": resolved.get("gripper_cmd"),
            "sync_policy": extra["sync_policy"],
            "gripper_sync_policy": extra["gripper_sync_policy"],
            "control_hz": args.control_hz,
            "action_source": extra["action_source"],
            "tf_frames": {
                "base": BASE_FRAME,
                "end_effector": LINK_CHAIN[-1],
            },
        },
        "action": {
            "type": "ee_delta_pose_gripper",
            "position_unit": "meter",
            "rotation_unit": "radian",
            "frame": BASE_FRAME,
            "layout": ACTION_LAYOUT,
            "gripper_open_value": 1.0,
            "gripper_closed_value": 0.0,
            "gripper_command_count": extra["gripper_command_count"],
            "gripper_event_override_count": extra["gripper_event_override_count"],
            "normalization": action_normalization_metadata(args.action_normalization),
        },
        "image": {
            "encoding": "rgb_uint8",
            "shape": extra["image_shape"],
            "wrist_shape": extra["wrist_image_shape"],
            "camera_view": "primary",
            "wrist_camera_view": "wrist" if extra["wrist_image_shape"] is not None else None,
            "video_fps": args.video_fps,
            "white_balance": {
                "enabled": bool(args.apply_white_balance),
                "applied": bool(white_balance_metadata),
                "config": white_balance_config,
                "gains_rgb": white_balance_metadata,
            },
        },
    }
    write_episode(out_dir, episode, metadata, args.video, args.video_fps)
    stats.append((bag_path, out_dir, len(episode), extra["action_source"]))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", nargs="?", default=DEFAULT_BAG_ROOT,
                        help=f"rosbag directory or parent directory (default: {DEFAULT_BAG_ROOT})")
    parser.add_argument("-o", "--output", default=DEFAULT_OUTPUT_ROOT,
                        help=f"output root directory (default: {DEFAULT_OUTPUT_ROOT})")
    parser.add_argument("--task-description", default="teleop task",
                        help="language task description stored in every step")
    parser.add_argument("--control-hz", type=float, default=20.0, help="output sampling rate")
    parser.add_argument("--video-fps", type=float, default=None,
                        help="episode.mp4 FPS (default: same as --control-hz)")
    parser.add_argument("--video", action=argparse.BooleanOptionalAction, default=True,
                        help="write episode.mp4 beside episode.npy (default: true)")
    parser.add_argument("--apply-white-balance", action=argparse.BooleanOptionalAction, default=True,
                        help="apply RGB gains from --white-balance-config to images (default: true)")
    parser.add_argument("--white-balance-config", default=DEFAULT_WHITE_BALANCE_CONFIG,
                        help=f"white balance YAML path (default: {DEFAULT_WHITE_BALANCE_CONFIG})")
    parser.add_argument("--action-min", nargs=len(ACTION_LAYOUT), type=float, default=[-0.2, -0.2, -0.2, -0.5, -0.5, -0.5, 0.0],
                        metavar="VALUE",
                        help="raw action lower bounds for dx dy dz droll dpitch dyaw gripper")
    parser.add_argument("--action-max", nargs=len(ACTION_LAYOUT), type=float, default=[0.2, 0.2, 0.2, 0.5, 0.5, 0.5, 1.0],
                        metavar="VALUE",
                        help="raw action upper bounds for dx dy dz droll dpitch dyaw gripper")
    parser.add_argument("--action-normalized-range", nargs=2, type=float, default=[-1.0, 1.0],
                        metavar=("MIN", "MAX"),
                        help="target range when --action-min/--action-max are set (default: -1 1)")
    parser.add_argument("--clip-normalized-action", action=argparse.BooleanOptionalAction, default=True,
                        help="clip normalized action to --action-normalized-range (default: true)")
    parser.add_argument("--default-gripper", type=float, default=1.0,
                        help="gripper value before the first command exists; 0.0=closed, 1.0=open")
    parser.add_argument("--gripper-event-max-dt", type=float, default=1.0,
                        help="max seconds to attach each sparse gripper command to an output step; "
                             "set 0 to disable event attachment")
    parser.add_argument("--success", action=argparse.BooleanOptionalAction, default=None,
                        help="episode success label; omitted by default")
    parser.add_argument("--score", type=float, default=None, help="episode score; omitted by default")
    return parser.parse_args()


def main():
    args = parse_args()
    task_slug = slugify(args.task_description)
    if args.video_fps is None:
        args.video_fps = args.control_hz
    args.action_normalization = resolve_action_normalization(args)

    bags = discover_bags(args.bag_path)
    if not bags:
        raise SystemExit(f"no rosbag directories found under {args.bag_path}")

    task_root = Path(args.output) / task_slug
    stats = []
    episode_id = 0
    for bag_path in bags:
        try:
            topic_types = get_topic_types(bag_path)
            resolved = resolve_topics(topic_types)
            wanted_topics = set(resolved.values())
            _, messages = read_bag(bag_path, wanted_topics)
            episode_dir = task_root / f"episode_{episode_id}"
            convert_bag(bag_path, episode_dir, episode_id, resolved, messages, args, stats)
            print(f"[ok] {bag_path} -> {episode_dir}")
            episode_id += 1
        except Exception as exc:
            print(f"[skip] {bag_path}: {exc}")

    if not stats:
        raise SystemExit("no bags converted successfully")

    print("\nConverted episodes:")
    for bag_path, out_dir, steps, action_source in stats:
        print(f"  {out_dir}: {steps} steps, action_source={action_source}, bag={bag_path}")


if __name__ == "__main__":
    main()
