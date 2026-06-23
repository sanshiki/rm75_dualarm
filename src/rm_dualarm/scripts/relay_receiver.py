#!/usr/bin/env python3
"""ROS 1 → ROS 2 topic relay receiver (host side).

Listens on a TCP port for JSON-serialized ROS messages forwarded from
a relay_sender running inside the ROS 1 Docker container, and publishes
them as native ROS 2 messages.

Usage (host):
    ros2 run rm_dualarm relay_receiver.py --ros-args -p port:=7654
"""

import json
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage


# Map topic → (ROS 2 message class, factory)
_TOPIC_MAP = {
    "/quest/joystick": Joy,
    "/quest/pose/headset": PoseStamped,
    "/tf": TFMessage,
    "/hand_points": None,  # skip, not needed
    "/quest/image": None,   # skip, large binary
    "/hand_pose": None,     # skip, dummy type
}


def _dict_to_joy(d):
    m = Joy()
    m.header.stamp.sec = d["header"]["stamp"]["sec"]
    m.header.stamp.nanosec = d["header"]["stamp"]["nanosec"]
    m.header.frame_id = d["header"]["frame_id"]
    m.axes = d["axes"]
    m.buttons = d["buttons"]
    return m


def _dict_to_pose_stamped(d):
    m = PoseStamped()
    m.header.stamp.sec = d["header"]["stamp"]["sec"]
    m.header.stamp.nanosec = d["header"]["stamp"]["nanosec"]
    m.header.frame_id = d["header"]["frame_id"]
    m.pose.position.x = d["pose"]["position"]["x"]
    m.pose.position.y = d["pose"]["position"]["y"]
    m.pose.position.z = d["pose"]["position"]["z"]
    m.pose.orientation.x = d["pose"]["orientation"]["x"]
    m.pose.orientation.y = d["pose"]["orientation"]["y"]
    m.pose.orientation.z = d["pose"]["orientation"]["z"]
    m.pose.orientation.w = d["pose"]["orientation"]["w"]
    return m


def _dict_to_tf_message(d):
    m = TFMessage()
    for t in d.get("transforms", []):
        ts = TransformStamped()
        ts.header.stamp.sec = t["header"]["stamp"]["sec"]
        ts.header.stamp.nanosec = t["header"]["stamp"]["nanosec"]
        ts.header.frame_id = t["header"]["frame_id"]
        ts.child_frame_id = t["child_frame_id"]
        ts.transform.translation.x = t["transform"]["translation"]["x"]
        ts.transform.translation.y = t["transform"]["translation"]["y"]
        ts.transform.translation.z = t["transform"]["translation"]["z"]
        ts.transform.rotation.x = t["transform"]["rotation"]["x"]
        ts.transform.rotation.y = t["transform"]["rotation"]["y"]
        ts.transform.rotation.z = t["transform"]["rotation"]["z"]
        ts.transform.rotation.w = t["transform"]["rotation"]["w"]
        m.transforms.append(ts)
    return m


_FACTORIES = {
    "sensor_msgs/Joy":           _dict_to_joy,
    "geometry_msgs/PoseStamped": _dict_to_pose_stamped,
    "tf2_msgs/TFMessage":        _dict_to_tf_message,
}


class RelayReceiver(Node):
    def __init__(self):
        super().__init__("relay_receiver")
        self.declare_parameter("port", 7654)
        self.declare_parameter("bind_ip", "0.0.0.0")
        port = self.get_parameter("port").value
        bind_ip = self.get_parameter("bind_ip").value

        self._pubs = {}
        for topic, cls in _TOPIC_MAP.items():
            if cls is not None:
                self._pubs[topic] = self.create_publisher(cls, topic, 10)

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((bind_ip, port))
        server.listen(1)
        self.get_logger().info(f"Relay listening on {bind_ip}:{port}")

        # Accept exactly one connection (blocking)
        self._conn, addr = server.accept()
        server.close()
        self.get_logger().info(f"Relay connected from {addr}")

        self._recv_buf = b""
        self._timer = self.create_timer(0.002, self._tick)  # ~500 Hz poll

    def _tick(self):
        """Read available data, parse complete JSON lines, publish."""
        try:
            chunk = self._conn.recv(65536)
            if not chunk:
                self.get_logger().error("Relay sender disconnected")
                self._timer.cancel()
                return
            self._recv_buf += chunk
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f"TCP recv error: {e}")
            self._timer.cancel()
            return

        # Process complete lines (each message is one JSON line ending with \n)
        while b"\n" in self._recv_buf:
            line, self._recv_buf = self._recv_buf.split(b"\n", 1)
            self._handle_line(line)

    def _handle_line(self, line):
        try:
            data = json.loads(line.decode("utf-8"))
        except Exception:
            self.get_logger().warn(f"Bad JSON: {line[:80]}")
            return

        topic = data.get("topic", "")
        msg_type = data.get("type", "")
        msg_dict = data.get("msg", {})

        pub = self._pubs.get(topic)
        if pub is None:
            return  # unknown/skipped topic

        factory = _FACTORIES.get(msg_type)
        if factory is None:
            return

        try:
            msg = factory(msg_dict)
            pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to build {msg_type}: {e}")


def main():
    rclpy.init()
    node = RelayReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
