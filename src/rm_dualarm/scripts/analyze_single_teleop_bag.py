#!/usr/bin/env python3
"""Offline analysis for single-arm teleop rosbag recordings.

Auto-detects sim vs real-hardware topics from the bag contents.
"""

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


# ---- Topic discovery ----
_FIXED = {
    "joint_states": "/joint_states",
    "tf": "/tf",
    "tf_static": "/tf_static",
}

_CMD_CANDIDATES = [
    "/rm_group_controller/joint_trajectory",   # sim (servo→controller)
    "/servo_bridge/joint_trajectory_in",         # real (servo→bridge)
    "/rm_driver/movej_canfd_cmd",                # real (bridge→driver)
]

_TARGET_CANDIDATES = ["/target_pose"]

EEF_FRAMES = ("base_link", [f"Link{i}" for i in range(1, 8)])


def _resolve_topics(bag_topics: set) -> dict:
    """Build a {logical_key: actual_topic} dict from what is in the bag."""
    resolved = {}
    for key, topic in _FIXED.items():
        if topic in bag_topics:
            resolved[key] = topic
    for t in _CMD_CANDIDATES:
        if t in bag_topics:
            resolved["cmd"] = t
            break
    for t in _TARGET_CANDIDATES:
        if t in bag_topics:
            resolved["target"] = t
            break
    return resolved


def read_bag(path, wanted_topics: set):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    messages = defaultdict(list)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in wanted_topics:
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


# ---- Frame builders ----

def joint_state_frame(rows):
    records = []
    for t, msg in rows:
        for name, pos in zip(msg.name, msg.position):
            records.append({"time": t, "joint": name, "position": pos})
    return pd.DataFrame(records)


def trajectory_frame(rows):
    """Parse JointTrajectory messages."""
    records = []
    for t, msg in rows:
        if not msg.points:
            continue
        pt = msg.points[0]
        for name, pos in zip(msg.joint_names, pt.positions):
            records.append({"time": t, "joint": name, "cmd_position": pos})
    return pd.DataFrame(records)


def jointpos_frame(rows):
    """Parse Jointpos messages (no joint names — inferred from ordering)."""
    records = []
    for t, msg in rows:
        for i, pos in enumerate(msg.joint, start=1):
            records.append({
                "time": t,
                "joint": f"joint{i}",
                "cmd_position": float(pos),
            })
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


# ---- TF / EEF ----

def _quat_to_rpy(qx, qy, qz, qw):
    r = R.from_quat([qx, qy, qz, qw])
    return R.from_matrix(r.as_matrix()).as_euler("xyz", degrees=False)


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
    rot = R.from_matrix(mat[:3, :3])
    quat = rot.as_quat()
    rpy = rot.as_euler("xyz", degrees=False)
    return {
        "time": t,
        "x": mat[0, 3],
        "y": mat[1, 3],
        "z": mat[2, 3],
        "qx": quat[0],
        "qy": quat[1],
        "qz": quat[2],
        "qw": quat[3],
        "roll": rpy[0],
        "pitch": rpy[1],
        "yaw": rpy[2],
    }


def eef_frame(tf_rows, tf_static_rows):
    base_frame, link_chain = EEF_FRAMES
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


# ---- Plotting ----

def plot_joint_tracking(out_dir, js_df, cmd_df):
    joints = [f"joint{i}" for i in range(1, 8)]
    has_js = "joint" in js_df.columns
    has_cmd = "joint" in cmd_df.columns
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for ax, joint in zip(axes, joints):
        js = js_df[js_df["joint"] == joint] if has_js else pd.DataFrame()
        cmd = cmd_df[cmd_df["joint"] == joint] if has_cmd else pd.DataFrame()
        if not js.empty:
            ax.plot(js["time"] - js["time"].iloc[0], js["position"], label="joint_state")
        if not cmd.empty:
            t0 = js["time"].iloc[0] if not js.empty else cmd["time"].iloc[0]
            ax.plot(cmd["time"] - t0, cmd["cmd_position"], label="trajectory_cmd", alpha=0.8)
        ax.set_ylabel(joint)
        ax.grid(True)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    path = os.path.join(out_dir, "joint_tracking.png")
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_pose_tracking(out_dir, target_df, eef_df):
    target_df = _add_rpy(target_df)
    for df in (target_df, eef_df):
        if df.empty or "time" not in df.columns:
            continue
        df.sort_values("time", inplace=True)
        for col in ("roll", "pitch", "yaw"):
            if col in df.columns:
                df[col] = np.unwrap(df[col].to_numpy())

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
    path = os.path.join(out_dir, "pose_tracking.png")
    fig.savefig(path)
    plt.close(fig)
    return path


# ---- Main ----

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    out_dir = args.output or os.path.join("analysis", os.path.basename(os.path.abspath(args.bag)))
    os.makedirs(out_dir, exist_ok=True)

    # -- 1. Peek at bag to discover topics --
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)
    bag_topics = {t.name: t.type for t in reader.get_all_topics_and_types()}

    resolved = _resolve_topics(set(bag_topics.keys()))
    wanted = set(resolved.values())
    print(f"Bag contains {len(bag_topics)} topics; matched {len(wanted)} for analysis:")
    for key in sorted(resolved):
        print(f"  {key:20s} → {resolved[key]}")

    # -- 2. Read only the resolved topics --
    messages = read_bag(args.bag, wanted)

    # -- 3. Build dataframes --
    js_df = joint_state_frame(messages.get(resolved.get("joint_states", ""), []))

    cmd_topic = resolved.get("cmd")
    cmd_rows = messages.get(cmd_topic, []) if cmd_topic else []
    if cmd_rows:
        msg_type = bag_topics.get(cmd_topic, "")
        if "Jointpos" in msg_type:
            cmd_df = jointpos_frame(cmd_rows)
        else:
            cmd_df = trajectory_frame(cmd_rows)
    else:
        cmd_df = pd.DataFrame()

    target_df = target_frame(messages.get(resolved.get("target", ""), []))

    eef_df = eef_frame(
        messages.get(resolved.get("tf", ""), []),
        messages.get(resolved.get("tf_static", ""), []),
    )

    # -- 4. Plot --
    outputs = []
    outputs.append(plot_joint_tracking(out_dir, js_df, cmd_df))
    outputs.append(plot_pose_tracking(out_dir, target_df, eef_df))

    # -- 5. Summary --
    summary = os.path.join(out_dir, "summary.md")
    with open(summary, "w", encoding="utf-8") as f:
        f.write("# Single Teleop Bag Analysis\n\n")
        f.write(f"**Bag:** `{args.bag}`\n\n")
        f.write("## Resolved Topics\n\n")
        for key in sorted(resolved):
            f.write(f"- `{key}` → `{resolved[key]}`\n")
        f.write("\n## Frequencies\n\n")
        for key in sorted(resolved):
            topic = resolved[key]
            freq = frequency(messages.get(topic, []))
            f.write(f"- `{topic}`: {freq:.2f} Hz\n")
        f.write("\n## Generated Plots\n\n")
        for path in outputs:
            f.write(f"- `{os.path.basename(path)}`\n")
        f.write("\n## Notes\n\n")
        f.write("- Joint tracking compares `/joint_states` against first-point trajectory/position commands.\n")
        f.write("- Pose tracking plots compare EEF TF against target pose per axis.\n")
        f.write("- Orientation is shown as RPY in radians.\n")
    print(f"Wrote analysis to {out_dir}")


if __name__ == "__main__":
    main()
