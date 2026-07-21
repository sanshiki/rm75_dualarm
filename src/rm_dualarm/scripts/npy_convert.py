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
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R


DEFAULT_BAG_ROOT = "bags/single"
DEFAULT_OUTPUT_ROOT = "convert_result"

FIXED_TOPICS = {
    "camera": "/camera/image_raw",
    "joint_states": "/joint_states",
    "tf": "/tf",
    "tf_static": "/tf_static",
}

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


def image_records(rows):
    records = []
    for t, msg in rows:
        records.append({"time": msg_time(t, msg), "image": image_to_rgb(msg)})
    return sorted(records, key=lambda item: item["time"])


def nearest(records, time_sec, max_dt=None):
    if not records:
        return None
    times = np.asarray([item["time"] for item in records], dtype=np.float64)
    idx = int(np.argmin(np.abs(times - time_sec)))
    if max_dt is not None and abs(times[idx] - time_sec) > max_dt:
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


def make_episode(messages, resolved, args):
    images = image_records(messages.get(resolved.get("camera", ""), []))
    eef = eef_pose_records(
        messages.get(resolved.get("tf", ""), []),
        messages.get(resolved.get("tf_static", ""), []),
    )
    targets = target_pose_records(messages.get(resolved.get("target_pose", ""), []))
    joints = joint_state_records(messages.get(resolved.get("joint_states", ""), []))
    traj_cmds = traj_cmd_records(messages.get(resolved.get("traj_cmd", ""), []))
    grippers = gripper_records(messages.get(resolved.get("gripper_cmd", ""), []))

    if not images:
        raise RuntimeError("missing camera images")
    if not eef:
        raise RuntimeError("missing end-effector TF")

    use_target_pose = bool(targets)
    times = sample_times(images, eef, targets if use_target_pose else [], args.control_hz)
    episode = []
    max_image_dt = 0.5 / float(args.control_hz)
    max_state_dt = 1.0 / float(args.control_hz)

    for index, time_sec in enumerate(times):
        image = nearest(images, time_sec, max_dt=max_image_dt)
        current = nearest(eef, time_sec, max_dt=max_state_dt)
        if image is None or current is None:
            continue

        target = nearest(targets, time_sec, max_dt=max_state_dt) if use_target_pose else None
        if target is None:
            if index + 1 >= len(times):
                continue
            target = nearest(eef, times[index + 1], max_dt=max_state_dt)
        if target is None:
            continue

        gripper = nearest(grippers, time_sec)
        gripper_value = float(gripper["value"]) if gripper else args.default_gripper
        action = np.concatenate([pose_delta(current, target), np.array([gripper_value], dtype=np.float32)])

        joint = nearest(joints, time_sec)
        traj_cmd = nearest(traj_cmds, time_sec)
        step = {
            "images": image["image"],
            "prompt": args.prompt,
            "action": action.astype(np.float32).tolist(),
            "timestamp": float(time_sec),
            "image_timestamp": float(image["time"]),
            "control_timestamp": float(time_sec),
            "ee_pose": np.concatenate([current["position"], current["quat"]]).astype(np.float32),
            "target_pose": np.concatenate([target["position"], target["quat"]]).astype(np.float32),
            "gripper_cmd": float(gripper_value),
        }
        if joint is not None:
            step["joint_state"] = joint["position"]
            step["joint_names"] = joint["name"]
        if traj_cmd is not None:
            step["traj_cmd"] = traj_cmd["position"]
        episode.append(step)

    if not episode:
        raise RuntimeError("no synchronized timesteps produced")

    duration = episode[-1]["timestamp"] - episode[0]["timestamp"] if len(episode) > 1 else 0.0
    image_shape = list(episode[0]["images"].shape)
    sync_policy = "control_time_nearest_neighbor"
    action_source = "target_pose_delta" if use_target_pose else "eef_next_pose_delta"
    return episode, {
        "duration_sec": float(duration),
        "image_shape": image_shape,
        "sync_policy": sync_policy,
        "action_source": action_source,
    }


def write_video_ffmpeg(path, episode, fps, ffmpeg_bin):
    if not episode:
        return "none"
    first = episode[0]["images"]
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
        for step in episode:
            rgb = step["images"]
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


def write_video_opencv(path, episode, fps):
    if not episode:
        return "none"
    first = episode[0]["images"]
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
        for step in episode:
            rgb = step["images"]
            if rgb.shape[:2] != (height, width):
                raise RuntimeError("all video frames must have the same resolution")
            writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    finally:
        writer.release()
    return codec


def write_video(path, episode, fps):
    ffmpeg_bin = shutil.which("ffmpeg")
    if ffmpeg_bin:
        return write_video_ffmpeg(path, episode, fps, ffmpeg_bin)
    print("[warn] ffmpeg not found; falling back to OpenCV video writer")
    return write_video_opencv(path, episode, fps)


def write_episode(out_dir, episode, metadata, write_mp4, fps):
    out_dir.mkdir(parents=True, exist_ok=True)
    np.save(out_dir / "episode.npy", np.asarray(episode, dtype=object), allow_pickle=True)
    if write_mp4:
        codec = write_video(out_dir / "episode.mp4", episode, fps)
        metadata["image"]["video_codec"] = codec
    with open(out_dir / "metadata.json", "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)


def convert_bag(bag_path, out_dir, episode_id, resolved, messages, args, stats):
    episode, extra = make_episode(messages, resolved, args)
    metadata = {
        "task": {
            "raw_name": args.task,
            "slug": args.task_slug,
            "prompt": args.prompt,
        },
        "episode": {
            "id": episode_id,
            "file": "episode.npy",
            "video": "episode.mp4" if args.video else None,
            "success": args.success,
            "score": args.score,
            "num_steps": len(episode),
            "duration_sec": extra["duration_sec"],
        },
        "source": {
            "type": "ros2_rosbag",
            "rosbag": str(bag_path.resolve()),
            "camera_topic": resolved.get("camera"),
            "tf_topic": resolved.get("tf"),
            "target_pose_topic": resolved.get("target_pose"),
            "traj_cmd_topic": resolved.get("traj_cmd"),
            "joint_states_topic": resolved.get("joint_states"),
            "gripper_cmd_topic": resolved.get("gripper_cmd"),
            "sync_policy": extra["sync_policy"],
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
            "layout": ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"],
            "gripper_open_value": 1.0,
            "gripper_closed_value": 0.0,
        },
        "image": {
            "encoding": "rgb_uint8",
            "shape": extra["image_shape"],
            "camera_view": "primary",
            "video_fps": args.video_fps,
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
    parser.add_argument("--task", default="teleop task", help="raw task name")
    parser.add_argument("--task-slug", default="", help="filesystem-safe task slug")
    parser.add_argument("--prompt", default="", help="language instruction stored in every step")
    parser.add_argument("--control-hz", type=float, default=20.0, help="output sampling rate")
    parser.add_argument("--video-fps", type=float, default=None,
                        help="episode.mp4 FPS (default: same as --control-hz)")
    parser.add_argument("--video", action=argparse.BooleanOptionalAction, default=True,
                        help="write episode.mp4 beside episode.npy (default: true)")
    parser.add_argument("--default-gripper", type=float, default=1.0,
                        help="gripper value when no command exists")
    parser.add_argument("--success", action=argparse.BooleanOptionalAction, default=None,
                        help="episode success label; omitted by default")
    parser.add_argument("--score", type=float, default=None, help="episode score; omitted by default")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.prompt:
        args.prompt = args.task
    args.task_slug = args.task_slug or slugify(args.task)
    if args.video_fps is None:
        args.video_fps = args.control_hz

    bags = discover_bags(args.bag_path)
    if not bags:
        raise SystemExit(f"no rosbag directories found under {args.bag_path}")

    task_root = Path(args.output) / args.task_slug
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
