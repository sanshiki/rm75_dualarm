#!/usr/bin/env python3
"""ROS 1 → ROS 2 topic relay sender (Docker side, ROS 1 noetic).

Subscribes to ROS 1 topics and forwards each message as a JSON line
over TCP to a relay_receiver on the host.

Usage (inside Docker container where rospy is available):
    python3 relay_sender.py --host 192.168.1.100 --port 7654
"""

import argparse
import json
import socket
import time
import threading

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage


def _joy_to_dict(msg):
    """sensor_msgs/Joy → dict."""
    return {
        "header": {
            "stamp": {"sec": msg.header.stamp.secs, "nanosec": msg.header.stamp.nsecs},
            "frame_id": msg.header.frame_id,
        },
        "axes": list(msg.axes),
        "buttons": list(msg.buttons),
    }


def _pose_stamped_to_dict(msg):
    """geometry_msgs/PoseStamped → dict."""
    return {
        "header": {
            "stamp": {"sec": msg.header.stamp.secs, "nanosec": msg.header.stamp.nsecs},
            "frame_id": msg.header.frame_id,
        },
        "pose": {
            "position": {"x": msg.pose.position.x, "y": msg.pose.position.y, "z": msg.pose.position.z},
            "orientation": {"x": msg.pose.orientation.x, "y": msg.pose.orientation.y,
                            "z": msg.pose.orientation.z, "w": msg.pose.orientation.w},
        },
    }


def _tf_message_to_dict(msg):
    """tf2_msgs/TFMessage → dict."""
    transforms = []
    for t in msg.transforms:
        transforms.append({
            "header": {
                "stamp": {"sec": t.header.stamp.secs, "nanosec": t.header.stamp.nsecs},
                "frame_id": t.header.frame_id,
            },
            "child_frame_id": t.child_frame_id,
            "transform": {
                "translation": {"x": t.transform.translation.x, "y": t.transform.translation.y,
                                "z": t.transform.translation.z},
                "rotation": {"x": t.transform.rotation.x, "y": t.transform.rotation.y,
                             "z": t.transform.rotation.z, "w": t.transform.rotation.w},
            },
        })
    return {"transforms": transforms}


# Topic → (message type, to-dict converter)
_RELAYS = [
    ("/quest/joystick",     Joy,           _joy_to_dict,          "sensor_msgs/Joy"),
    ("/quest/pose/headset", PoseStamped,   _pose_stamped_to_dict, "geometry_msgs/PoseStamped"),
    ("/tf",                 TFMessage,     _tf_message_to_dict,   "tf2_msgs/TFMessage"),
]


class RelaySender:
    def __init__(self, host, port):
        self._host = host
        self._port = port
        self._sock = None
        self._lock = threading.Lock()
        self._connect()
        self._setup_subscribers()
        rospy.loginfo(f"Relay sender ready → {host}:{port}")

    def _connect(self):
        while self._sock is None and not rospy.is_shutdown():
            try:
                self._sock = socket.create_connection((self._host, self._port), timeout=5)
                self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except (socket.error, OSError):
                rospy.logwarn(f"Waiting for receiver at {self._host}:{self._port} ...")
                if rospy.is_shutdown():
                    break
                time.sleep(2)

    def _setup_subscribers(self):
        for topic, msg_cls, converter, msg_type in _RELAYS:
            rospy.Subscriber(
                topic, msg_cls,
                lambda m, t=topic, c=converter, mt=msg_type: self._on_msg(t, mt, c(m)),
                queue_size=10,
            )

    def _on_msg(self, topic, msg_type, msg_dict):
        """Convert ROS msg → dict → JSON, send over TCP."""
        frame = {"topic": topic, "type": msg_type, "msg": msg_dict}
        try:
            line = json.dumps(frame) + "\n"
            with self._lock:
                self._sock.sendall(line.encode("utf-8"))
        except (socket.error, OSError) as e:
            rospy.logerr(f"TCP send error: {e}")
            self._sock = None
            self._connect()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7654)
    args = parser.parse_args()

    rospy.init_node("relay_sender", anonymous=True)
    RelaySender(host=args.host, port=args.port)
    rospy.spin()


if __name__ == "__main__":
    main()
