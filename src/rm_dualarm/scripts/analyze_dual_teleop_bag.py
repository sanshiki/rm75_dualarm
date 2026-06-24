#!/usr/bin/env python3
"""Offline analysis for dual-arm teleop rosbag recordings."""

import argparse
import os
from collections import defaultdict

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R


TOPICS = {
    "joint_states": "/joint_states",
    "left_target": "/left/target_pose",
    "right_target": "/right/target_pose",
    "left_cmd": "/left_rm_group_controller/joint_trajectory",
    "right_cmd": "/right_rm_group_controller/joint_trajectory",
    "tf": "/tf",
    "tf_static": "/tf_static",
}

EEF_FRAMES = {
    "left": ("left_base_link", [f"left_Link{i}" for i in range(1, 8)]),
    "right": ("right_base_link", [f"right_Link{i}" for i in range(1, 8)]),
}


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    messages = defaultdict(list)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in TOPICS.values():
            continue
        msg_type = get_message(types[topic])
        msg = deserialize_message(data, msg_type)
        messages[topic].append((stamp * 1e-9, msg))
    return messages


def frequency(rows):
    if len(rows) < 2:
        return 0.0
    times = np.array([t for t, _ in rows])
    diffs = np.diff(times)
    diffs = diffs[diffs > 1e-6]
    return float(1.0 / np.mean(diffs)) if len(diffs) else 0.0


def joint_state_frame(rows):
    records = []
    for t, msg in rows:
        for name, pos in zip(msg.name, msg.position):
            records.append({"time": t, "joint": name, "position": pos})
    return pd.DataFrame(records)


def trajectory_frame(rows):
    records = []
    for t, msg in rows:
        if not msg.points:
            continue
        pt = msg.points[0]
        for name, pos in zip(msg.joint_names, pt.positions):
            records.append({"time": t, "joint": name, "cmd_position": pos})
    return pd.DataFrame(records)


def target_frame(rows):
    records = []
    for t, msg in rows:
        records.append({
            "time": t,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "qx": msg.pose.orientation.x,
            "qy": msg.pose.orientation.y,
            "qz": msg.pose.orientation.z,
            "qw": msg.pose.orientation.w,
        })
    return pd.DataFrame(records)


def _quat_to_rpy(qx, qy, qz, qw):
    return R.from_quat([qx, qy, qz, qw]).as_euler("xyz", degrees=False)


def _add_rpy(df):
    if df.empty:
        return df
    rpy = np.array([
        _quat_to_rpy(row.qx, row.qy, row.qz, row.qw)
        for row in df.itertuples()
    ])
    df = df.copy()
    df["roll"] = rpy[:, 0]
    df["pitch"] = rpy[:, 1]
    df["yaw"] = rpy[:, 2]
    return df


def _transform_matrix(trans):
    p = trans.transform.translation
    q = trans.transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[:3, 3] = [p.x, p.y, p.z]
    return mat


def _matrix_record(t, mat):
    quat = R.from_matrix(mat[:3, :3]).as_quat()
    roll, pitch, yaw = R.from_matrix(mat[:3, :3]).as_euler("xyz", degrees=False)
    return {
        "time": t,
        "x": mat[0, 3],
        "y": mat[1, 3],
        "z": mat[2, 3],
        "qx": quat[0],
        "qy": quat[1],
        "qz": quat[2],
        "qw": quat[3],
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def eef_frame(tf_rows, tf_static_rows, side):
    base_frame, link_chain = EEF_FRAMES[side]
    ee_frame = link_chain[-1]
    records = []
    static_edges = {}
    for _, msg in tf_static_rows:
        for trans in msg.transforms:
            static_edges[(trans.header.frame_id, trans.child_frame_id)] = _transform_matrix(trans)

    latest_edges = dict(static_edges)
    chain_edges = []
    parent = base_frame
    for child in link_chain:
        chain_edges.append((parent, child))
        parent = child

    for t, msg in tf_rows:
        for trans in msg.transforms:
            latest_edges[(trans.header.frame_id, trans.child_frame_id)] = _transform_matrix(trans)
            if trans.header.frame_id == base_frame and trans.child_frame_id == ee_frame:
                records.append(_matrix_record(t, latest_edges[(base_frame, ee_frame)]))
                continue

        if all(edge in latest_edges for edge in chain_edges):
            mat = np.eye(4)
            for edge in chain_edges:
                mat = mat @ latest_edges[edge]
            records.append(_matrix_record(t, mat))
    return pd.DataFrame(records)


def plot_joint_tracking(out_dir, side, js_df, cmd_df):
    prefix = f"{side}_joint"
    joints = [f"{side}_joint{i}" for i in range(1, 8)]
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for ax, joint in zip(axes, joints):
        js = js_df[js_df["joint"] == joint]
        cmd = cmd_df[cmd_df["joint"] == joint]
        if not js.empty:
            ax.plot(js["time"] - js["time"].iloc[0], js["position"], label="joint_state")
        if not cmd.empty:
            t0 = cmd["time"].iloc[0] if js.empty else js["time"].iloc[0]
            ax.plot(cmd["time"] - t0, cmd["cmd_position"], label="trajectory_cmd", alpha=0.8)
        ax.set_ylabel(joint)
        ax.grid(True)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    path = os.path.join(out_dir, f"joint_tracking_{side}.png")
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_pose_tracking(out_dir, side, target_df, eef_df):
    target_df = _add_rpy(target_df)
    fig, axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)
    axes_info = [
        ("x", "position x [m]"),
        ("y", "position y [m]"),
        ("z", "position z [m]"),
        ("roll", "roll [rad]"),
        ("pitch", "pitch [rad]"),
        ("yaw", "yaw [rad]"),
    ]
    t0_candidates = []
    if not target_df.empty:
        t0_candidates.append(target_df["time"].iloc[0])
    if not eef_df.empty:
        t0_candidates.append(eef_df["time"].iloc[0])
    t0 = min(t0_candidates) if t0_candidates else 0.0

    for ax, (field, ylabel) in zip(axes, axes_info):
        if not target_df.empty:
            ax.plot(
                target_df["time"] - t0,
                target_df[field],
                label=f"target {field}",
                linewidth=1.4,
            )
        if not eef_df.empty:
            ax.plot(
                eef_df["time"] - t0,
                eef_df[field],
                label=f"eef {field}",
                linewidth=1.2,
                alpha=0.85,
            )
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right")
        ax.grid(True)
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    path = os.path.join(out_dir, f"pose_tracking_{side}.png")
    fig.savefig(path)
    plt.close(fig)
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    out_dir = args.output or os.path.join("analysis", os.path.basename(os.path.abspath(args.bag)))
    os.makedirs(out_dir, exist_ok=True)

    messages = read_bag(args.bag)
    js_df = joint_state_frame(messages[TOPICS["joint_states"]])
    left_cmd_df = trajectory_frame(messages[TOPICS["left_cmd"]])
    right_cmd_df = trajectory_frame(messages[TOPICS["right_cmd"]])
    left_target_df = target_frame(messages[TOPICS["left_target"]])
    right_target_df = target_frame(messages[TOPICS["right_target"]])
    left_eef_df = eef_frame(messages[TOPICS["tf"]], messages[TOPICS["tf_static"]], "left")
    right_eef_df = eef_frame(messages[TOPICS["tf"]], messages[TOPICS["tf_static"]], "right")

    outputs = []
    outputs.append(plot_joint_tracking(out_dir, "left", js_df, left_cmd_df))
    outputs.append(plot_joint_tracking(out_dir, "right", js_df, right_cmd_df))
    outputs.append(plot_pose_tracking(out_dir, "left", left_target_df, left_eef_df))
    outputs.append(plot_pose_tracking(out_dir, "right", right_target_df, right_eef_df))

    summary = os.path.join(out_dir, "summary.md")
    with open(summary, "w", encoding="utf-8") as f:
        f.write("# Dual Teleop Bag Analysis\n\n")
        f.write("## Frequencies\n\n")
        for key, topic in TOPICS.items():
            f.write(f"- `{topic}`: {frequency(messages[topic]):.2f} Hz\n")
        f.write("\n## Generated Plots\n\n")
        for path in outputs:
            f.write(f"- `{os.path.basename(path)}`\n")
        f.write("\n## Notes\n\n")
        f.write("- Joint tracking compares `/joint_states` against first-point trajectory commands.\n")
        f.write("- Pose tracking plots compare EEF TF against target pose per axis.\n")
        f.write("- Orientation is shown as RPY in radians.\n")
    print(f"Wrote analysis to {out_dir}")


if __name__ == "__main__":
    main()
