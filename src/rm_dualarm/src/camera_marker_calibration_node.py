#!/usr/bin/python3
"""AprilTag/ArUco camera pose capture and drift checker."""

import math
import os
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


ARUCO_DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


class CaptureComplete(Exception):
    """Raised to leave rclpy.spin after a one-shot capture succeeds."""


def _pose_to_msg(position, quat, frame_id, stamp):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(position[0])
    msg.pose.position.y = float(position[1])
    msg.pose.position.z = float(position[2])
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


def _transform_to_msg(position, quat, parent_frame, child_frame, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = float(position[0])
    msg.transform.translation.y = float(position[1])
    msg.transform.translation.z = float(position[2])
    msg.transform.rotation.x = float(quat[0])
    msg.transform.rotation.y = float(quat[1])
    msg.transform.rotation.z = float(quat[2])
    msg.transform.rotation.w = float(quat[3])
    return msg


def _normalize_quat(quat):
    quat = np.asarray(quat, dtype=float)
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-12:
        return None
    return quat / norm


def _average_quat(quats):
    rotations = R.from_quat(np.asarray(quats, dtype=float))
    return rotations.mean().as_quat()


def _pose_matrix(position, quat):
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat(quat).as_matrix()
    matrix[:3, 3] = np.asarray(position, dtype=float)
    return matrix


def _quat_from_rvec(rvec):
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    if not np.all(np.isfinite(rotation_matrix)):
        return None
    return _normalize_quat(R.from_matrix(rotation_matrix).as_quat())


def _matrix_pose(matrix):
    return matrix[:3, 3], R.from_matrix(matrix[:3, :3]).as_quat()


def _pose_yaml(position, quat):
    return {
        "position": [float(v) for v in position],
        "orientation_quat": [float(v) for v in quat],
    }


class CameraMarkerCalibration(Node):
    """Capture or compare camera pose relative to one planar marker."""

    def __init__(self):
        super().__init__("camera_marker_calibration")

        self.declare_parameter("mode", "check")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("marker_dictionary", "DICT_APRILTAG_36h11")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size_m", 0.08)
        self.declare_parameter("marker_frame", "calibration_marker")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("approximate_horizontal_fov_deg", 0.0)
        self.declare_parameter("output_file", "/tmp/camera_marker_calibration.yaml")
        self.declare_parameter("reference_file", "")
        self.declare_parameter("sample_count", 50)
        self.declare_parameter("min_valid_samples", 20)
        self.declare_parameter("status_period", 1.0)
        self.declare_parameter("translation_warn_mm", 10.0)
        self.declare_parameter("rotation_warn_deg", 2.0)
        self.declare_parameter("log_check_status", True)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("exit_after_capture", True)

        self._mode = str(self.get_parameter("mode").value).lower()
        if self._mode not in ("capture", "check"):
            raise ValueError("mode must be 'capture' or 'check'")

        dictionary_name = self.get_parameter("marker_dictionary").value
        if dictionary_name not in ARUCO_DICTIONARIES:
            valid_names = ", ".join(sorted(ARUCO_DICTIONARIES.keys()))
            raise ValueError(f"Unsupported marker_dictionary '{dictionary_name}'. Valid: {valid_names}")

        self._dictionary_name = dictionary_name
        self._dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dictionary_name])
        if hasattr(cv2.aruco, "DetectorParameters"):
            self._detector_params = cv2.aruco.DetectorParameters()
        else:
            self._detector_params = cv2.aruco.DetectorParameters_create()
        self._detector = (
            cv2.aruco.ArucoDetector(self._dictionary, self._detector_params)
            if hasattr(cv2.aruco, "ArucoDetector")
            else None
        )

        self._marker_id = int(self.get_parameter("marker_id").value)
        self._marker_size = float(self.get_parameter("marker_size_m").value)
        if self._marker_size <= 0.0:
            raise ValueError("marker_size_m must be positive")

        self._marker_frame = self.get_parameter("marker_frame").value
        self._camera_frame_param = self.get_parameter("camera_frame").value
        self._camera_frame = self._camera_frame_param
        self._approximate_horizontal_fov_deg = float(
            self.get_parameter("approximate_horizontal_fov_deg").value)
        self._output_file = self.get_parameter("output_file").value
        self._reference_file = self.get_parameter("reference_file").value or self._output_file
        self._sample_count = int(self.get_parameter("sample_count").value)
        self._min_valid_samples = int(self.get_parameter("min_valid_samples").value)
        self._status_period = float(self.get_parameter("status_period").value)
        self._translation_warn_mm = float(self.get_parameter("translation_warn_mm").value)
        self._rotation_warn_deg = float(self.get_parameter("rotation_warn_deg").value)
        self._log_check_status = bool(self.get_parameter("log_check_status").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._exit_after_capture = bool(self.get_parameter("exit_after_capture").value)

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None
        self._warned_invalid_camera_info = False
        self._samples = []
        self._reference_pose = None
        self._last_status_time = self.get_clock().now()
        self._finished_capture = False
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self._current_pose_pub = self.create_publisher(PoseStamped, "/camera_marker/current_pose", 10)
        self._reference_pose_pub = self.create_publisher(PoseStamped, "/camera_marker/reference_pose", 10)
        self._delta_pose_pub = self.create_publisher(PoseStamped, "/camera_marker/delta_pose", 10)
        self._status_pub = self.create_publisher(String, "/camera_marker/status", 10)

        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_cb,
            10,
        )

        if self._mode == "check":
            self._reference_pose = self._load_reference(self._reference_file)

        self.get_logger().info(
            f"Camera marker calibration started in {self._mode} mode, "
            f"dictionary={self._dictionary_name}, marker_id={self._marker_id}, "
            f"marker_size_m={self._marker_size:.4f}"
        )

    def _camera_info_cb(self, msg):
        if self._camera_matrix is None:
            camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
            dist_coeffs = np.array(msg.d, dtype=float)
            if not self._valid_camera_matrix(camera_matrix):
                if self._approximate_horizontal_fov_deg > 0.0:
                    camera_matrix, dist_coeffs = self._approximate_intrinsics(msg)
                    self.get_logger().warn(
                        "CameraInfo has no valid intrinsics; using approximate "
                        f"horizontal FOV {self._approximate_horizontal_fov_deg:.1f} deg. "
                        "Use a calibrated camera_info_url for accurate pose."
                    )
                else:
                    self._publish_status(
                        "invalid_camera_info: provide camera_info_url or set "
                        "approximate_horizontal_fov_deg for a rough test"
                    )
                    if not self._warned_invalid_camera_info:
                        self.get_logger().error(
                            "CameraInfo intrinsics are invalid/all zero. Provide a calibrated "
                            "camera_info_url to camera.launch.py, or pass "
                            "approximate_horizontal_fov_deg for a rough test."
                        )
                        self._warned_invalid_camera_info = True
                    return
            self._camera_matrix = camera_matrix
            self._dist_coeffs = dist_coeffs
            if not self._camera_frame:
                self._camera_frame = msg.header.frame_id
            self.get_logger().info(
                f"Loaded camera intrinsics from {msg.header.frame_id or 'CameraInfo'}"
            )

    def _valid_camera_matrix(self, camera_matrix):
        return (
            camera_matrix.shape == (3, 3)
            and np.all(np.isfinite(camera_matrix))
            and camera_matrix[0, 0] > 0.0
            and camera_matrix[1, 1] > 0.0
            and camera_matrix[2, 2] != 0.0
        )

    def _approximate_intrinsics(self, msg):
        width = float(msg.width)
        height = float(msg.height)
        fov_rad = math.radians(self._approximate_horizontal_fov_deg)
        fx = width / (2.0 * math.tan(fov_rad * 0.5))
        fy = fx
        cx = (width - 1.0) * 0.5
        cy = (height - 1.0) * 0.5
        return (
            np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=float),
            np.zeros(5, dtype=float),
        )

    def _image_cb(self, msg):
        if self._camera_matrix is None:
            self._publish_status("waiting_for_camera_info")
            return
        if self._finished_capture:
            return

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return

        estimate = self._estimate_marker_pose(image)
        if estimate is None:
            self._publish_status("marker_not_detected", throttle=self._mode == "check")
            return

        position, quat = estimate
        stamp = msg.header.stamp
        frame_id = self._marker_frame
        camera_frame = self._camera_frame or msg.header.frame_id or "camera_optical_frame"
        current_msg = _pose_to_msg(position, quat, frame_id, stamp)
        self._current_pose_pub.publish(current_msg)

        if self._tf_broadcaster is not None:
            self._tf_broadcaster.sendTransform(
                _transform_to_msg(position, quat, self._marker_frame, camera_frame, stamp)
            )

        if self._mode == "capture":
            self._handle_capture(position, quat)
        else:
            self._handle_check(position, quat, stamp)

    def _estimate_marker_pose(self, image) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                image,
                self._dictionary,
                parameters=self._detector_params,
            )
        if ids is None:
            return None

        ids_flat = ids.flatten()
        matches = np.where(ids_flat == self._marker_id)[0]
        if len(matches) == 0:
            return None

        marker_corners = corners[int(matches[0])].reshape(4, 2).astype(np.float32)
        half = self._marker_size * 0.5
        object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        solve_flags = [cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE]
        for flag in solve_flags:
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                marker_corners,
                self._camera_matrix,
                self._dist_coeffs,
                flags=flag,
            )
            if not ok:
                continue

            position = np.asarray(tvec, dtype=float).reshape(3)
            quat = _quat_from_rvec(rvec)
            if (
                quat is not None
                and np.all(np.isfinite(position))
                and np.linalg.norm(position) > 1e-9
            ):
                return position, quat

        return None

    def _handle_capture(self, position, quat):
        self._samples.append((position, quat))
        self._publish_status(f"capturing {len(self._samples)}/{self._sample_count}")
        if len(self._samples) < self._sample_count:
            return

        if len(self._samples) < self._min_valid_samples:
            self.get_logger().error(
                f"Capture failed: only {len(self._samples)} valid samples, "
                f"need at least {self._min_valid_samples}"
            )
            self._finished_capture = True
            return

        positions = [sample[0] for sample in self._samples]
        quats = [sample[1] for sample in self._samples]
        position_mean = np.mean(positions, axis=0)
        quat_mean = _average_quat(quats)
        data = {
            "schema_version": 1,
            "description": "Camera pose relative to a single planar AprilTag/ArUco marker.",
            "mode": "capture",
            "sample_count": len(self._samples),
            "frames": {
                "marker_frame": self._marker_frame,
                "camera_frame": self._camera_frame,
            },
            "marker": {
                "dictionary": self._dictionary_name,
                "id": self._marker_id,
                "size_m": self._marker_size,
            },
            "marker_to_camera": _pose_yaml(position_mean, quat_mean),
        }

        os.makedirs(os.path.dirname(os.path.abspath(self._output_file)), exist_ok=True)
        with open(self._output_file, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
        self._finished_capture = True
        self._publish_status(f"capture_saved {self._output_file}")
        self.get_logger().info(
            f"Saved camera marker calibration to {self._output_file} "
            f"({len(self._samples)} samples)"
        )
        if self._exit_after_capture:
            raise CaptureComplete()

    def _handle_check(self, position, quat, stamp):
        reference_position, reference_quat = self._reference_pose
        reference_msg = _pose_to_msg(reference_position, reference_quat, self._marker_frame, stamp)
        self._reference_pose_pub.publish(reference_msg)

        current_matrix = _pose_matrix(position, quat)
        reference_matrix = _pose_matrix(reference_position, reference_quat)
        delta_matrix = np.linalg.inv(reference_matrix) @ current_matrix
        delta_position, delta_quat = _matrix_pose(delta_matrix)

        translation_error_m = float(np.linalg.norm(delta_position))
        translation_error_mm = translation_error_m * 1000.0
        rotation_error_deg = float(R.from_quat(delta_quat).magnitude() * 180.0 / math.pi)
        delta_msg = _pose_to_msg(delta_position, delta_quat, self._marker_frame, stamp)
        self._delta_pose_pub.publish(delta_msg)

        state = "ok"
        if (
            translation_error_mm > self._translation_warn_mm
            or rotation_error_deg > self._rotation_warn_deg
        ):
            state = "warn"
        self._publish_status(
            f"{state} translation_error_mm={translation_error_mm:.2f} "
            f"rotation_error_deg={rotation_error_deg:.2f} "
            f"pose_xyz_m=[{position[0]:.4f},{position[1]:.4f},{position[2]:.4f}] "
            f"delta_xyz_mm=[{delta_position[0] * 1000.0:.1f},"
            f"{delta_position[1] * 1000.0:.1f},{delta_position[2] * 1000.0:.1f}]",
            throttle=True,
        )

    def _load_reference(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"reference_file does not exist: {path}")

        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        marker = data.get("marker", {})
        if marker.get("dictionary") and marker["dictionary"] != self._dictionary_name:
            self.get_logger().warn(
                f"Reference dictionary is {marker['dictionary']}, "
                f"current parameter is {self._dictionary_name}"
            )
        if marker.get("id") is not None and int(marker["id"]) != self._marker_id:
            self.get_logger().warn(
                f"Reference marker id is {marker['id']}, current parameter is {self._marker_id}"
            )
        if marker.get("size_m") is not None and abs(float(marker["size_m"]) - self._marker_size) > 1e-6:
            self.get_logger().warn(
                f"Reference marker size is {marker['size_m']}, current parameter is {self._marker_size}"
            )

        pose = data.get("marker_to_camera")
        if not pose:
            raise ValueError(f"reference_file missing marker_to_camera: {path}")

        position = np.asarray(pose["position"], dtype=float)
        quat = _normalize_quat(pose["orientation_quat"])
        if quat is None:
            raise ValueError(f"reference_file contains an invalid quaternion: {path}")
        self.get_logger().info(f"Loaded camera marker reference from {path}")
        return position, quat

    def _publish_status(self, text, throttle=False):
        if throttle:
            now = self.get_clock().now()
            elapsed = (now - self._last_status_time).nanoseconds / 1e9
            if elapsed < max(self._status_period, 0.1):
                return
            self._last_status_time = now

        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        if text.startswith("warn") or text.startswith("capture_saved"):
            self.get_logger().warn(text) if text.startswith("warn") else self.get_logger().info(text)
        elif throttle and self._log_check_status:
            self.get_logger().info(text)
        elif not throttle:
            self.get_logger().debug(text)


def main(args=None):
    rclpy.init(args=args)
    node = CameraMarkerCalibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, CaptureComplete):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
