# ================================================================
# RealSense D435i wrist camera bringup
# ================================================================
# Starts wrist-mounted RealSense cameras and publishes the fixed TF
# from the arm end-effector link to each camera root frame.
#
# Usage:
#   ros2 launch rm_dualarm wrist_cameras.launch.py control_mode:=single single_serial_no:=<serial>
#   ros2 launch rm_dualarm wrist_cameras.launch.py control_mode:=dual left_serial_no:=<serial> right_serial_no:=<serial>
#
# The *_xyz and *_rpy arguments are static mount transforms relative
# to Link7 / left_Link7 / right_Link7. Defaults are zero until measured.
# ================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _triplet_default(values):
    return " ".join(str(value) for value in values)


def _load_defaults(pkg_share):
    defaults = {
        "control_mode": "single",
        "single_wrist_side": "right",
        "single_serial_no": "''",
        "left_serial_no": "''",
        "right_serial_no": "''",
        "single_parent_frame": "Link7",
        "single_camera_link": "wrist_camera_link",
        "left_parent_frame": "left_Link7",
        "left_camera_link": "left_wrist_camera_link",
        "right_parent_frame": "right_Link7",
        "right_camera_link": "right_wrist_camera_link",
        "left_camera_xyz": "0.0 0.0 0.0",
        "left_camera_rpy": "0.0 0.0 0.0",
        "right_camera_xyz": "0.0 0.0 0.0",
        "right_camera_rpy": "0.0 0.0 0.0",
        "enable_color": "true",
        "enable_depth": "true",
        "enable_gyro": "false",
        "enable_accel": "false",
        "align_depth": "false",
        "pointcloud": "false",
        "color_profile": "640x480x30",
        "color_format": "RGB8",
        "color_auto_exposure": "false",
        "rgb_camera_exposure": "100",
        "depth_profile": "640x480x30",
        "depth_auto_exposure": "false",
        "gyro_fps": "200",
        "accel_fps": "63",
        "unite_imu_method": "2",
        "enable_sync": "false",
    }
    params_path = os.path.join(pkg_share, "config", "wrist_camera_params.yaml")
    if not os.path.exists(params_path):
        return defaults

    with open(params_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    defaults["control_mode"] = str(cfg.get("control_mode", defaults["control_mode"]))
    defaults["single_wrist_side"] = str(cfg.get("single_wrist_side", defaults["single_wrist_side"]))

    serials = cfg.get("serial_numbers", {}) or {}
    defaults["single_serial_no"] = str(serials.get("single", defaults["single_serial_no"]))
    defaults["left_serial_no"] = str(serials.get("left", defaults["left_serial_no"]))
    defaults["right_serial_no"] = str(serials.get("right", defaults["right_serial_no"]))

    mounts = cfg.get("mounts", {}) or {}
    single_mount = mounts.get("single", {}) or {}
    defaults["single_parent_frame"] = str(single_mount.get("parent_frame", defaults["single_parent_frame"]))
    defaults["single_camera_link"] = str(single_mount.get("camera_link", defaults["single_camera_link"]))
    for side in ("left", "right"):
        mount = mounts.get(side, {}) or {}
        defaults[f"{side}_parent_frame"] = str(
            mount.get("parent_frame", defaults[f"{side}_parent_frame"])
        )
        defaults[f"{side}_camera_link"] = str(
            mount.get("camera_link", defaults[f"{side}_camera_link"])
        )
        defaults[f"{side}_camera_xyz"] = _triplet_default(
            mount.get("xyz", defaults[f"{side}_camera_xyz"].split())
        )
        defaults[f"{side}_camera_rpy"] = _triplet_default(
            mount.get("rpy", defaults[f"{side}_camera_rpy"].split())
        )

    streams = cfg.get("streams", {}) or {}
    for name in ("enable_color", "enable_depth", "enable_gyro", "enable_accel", "align_depth", "pointcloud"):
        if name in streams:
            defaults[name] = str(streams[name]).lower()

    realsense = cfg.get("realsense", {}) or {}
    realsense_map = {
        "color_profile": "rgb_camera.color_profile",
        "color_format": "rgb_camera.color_format",
        "color_auto_exposure": "rgb_camera.enable_auto_exposure",
        "rgb_camera_exposure": "rgb_camera.exposure",
        "depth_profile": "depth_module.depth_profile",
        "depth_auto_exposure": "depth_module.enable_auto_exposure",
        "gyro_fps": "gyro_fps",
        "accel_fps": "accel_fps",
        "unite_imu_method": "unite_imu_method",
        "enable_sync": "enable_sync",
    }
    for local_name, realsense_name in realsense_map.items():
        if realsense_name in realsense:
            value = realsense[realsense_name]
            defaults[local_name] = str(value).lower() if isinstance(value, bool) else str(value)

    return defaults


def _split_triplet(context, name):
    value = LaunchConfiguration(name).perform(context)
    parts = value.replace(",", " ").split()
    if len(parts) != 3:
        raise RuntimeError(f"{name} must contain exactly 3 values, got: {value!r}")
    return parts


def _static_tf(context, parent_frame, child_frame, xyz_arg, rpy_arg):
    xyz = _split_triplet(context, xyz_arg)
    rpy = _split_triplet(context, rpy_arg)
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{child_frame}_static_tf",
        output="screen",
        arguments=[
            "--x", xyz[0],
            "--y", xyz[1],
            "--z", xyz[2],
            "--roll", rpy[0],
            "--pitch", rpy[1],
            "--yaw", rpy[2],
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
    )


def _realsense_camera(
    rs_launch_path,
    namespace,
    camera_name,
    serial_no,
    base_frame_id,
    enable_color,
    enable_depth,
    enable_gyro,
    enable_accel,
    align_depth_enable,
    pointcloud_enable,
    rgb_camera_color_profile,
    rgb_camera_color_format,
    rgb_camera_enable_auto_exposure,
    rgb_camera_exposure,
    depth_module_depth_profile,
    depth_module_enable_auto_exposure,
    gyro_fps,
    accel_fps,
    unite_imu_method,
    enable_sync,
):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            "camera_namespace": namespace,
            "camera_name": camera_name,
            "serial_no": serial_no,
            "device_type": "d435i",
            "base_frame_id": base_frame_id,
            "enable_color": enable_color,
            "enable_depth": enable_depth,
            "enable_gyro": enable_gyro,
            "enable_accel": enable_accel,
            "align_depth.enable": align_depth_enable,
            "pointcloud.enable": pointcloud_enable,
            "rgb_camera.color_profile": rgb_camera_color_profile,
            "rgb_camera.color_format": rgb_camera_color_format,
            "rgb_camera.enable_auto_exposure": rgb_camera_enable_auto_exposure,
            "rgb_camera.exposure": rgb_camera_exposure,
            "depth_module.depth_profile": depth_module_depth_profile,
            "depth_module.enable_auto_exposure": depth_module_enable_auto_exposure,
            "gyro_fps": gyro_fps,
            "accel_fps": accel_fps,
            "unite_imu_method": unite_imu_method,
            "enable_sync": enable_sync,
        }.items(),
    )


def _setup(context, *args, **kwargs):
    control_mode = LaunchConfiguration("control_mode").perform(context)
    single_wrist_side = LaunchConfiguration("single_wrist_side").perform(context)
    rs_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    common = {
        "enable_color": LaunchConfiguration("enable_color"),
        "enable_depth": LaunchConfiguration("enable_depth"),
        "enable_gyro": LaunchConfiguration("enable_gyro"),
        "enable_accel": LaunchConfiguration("enable_accel"),
        "align_depth_enable": LaunchConfiguration("align_depth.enable"),
        "pointcloud_enable": LaunchConfiguration("pointcloud.enable"),
        "rgb_camera_color_profile": LaunchConfiguration("rgb_camera.color_profile"),
        "rgb_camera_color_format": LaunchConfiguration("rgb_camera.color_format"),
        "rgb_camera_enable_auto_exposure": LaunchConfiguration("rgb_camera.enable_auto_exposure"),
        "rgb_camera_exposure": LaunchConfiguration("rgb_camera.exposure"),
        "depth_module_depth_profile": LaunchConfiguration("depth_module.depth_profile"),
        "depth_module_enable_auto_exposure": LaunchConfiguration("depth_module.enable_auto_exposure"),
        "gyro_fps": LaunchConfiguration("gyro_fps"),
        "accel_fps": LaunchConfiguration("accel_fps"),
        "unite_imu_method": LaunchConfiguration("unite_imu_method"),
        "enable_sync": LaunchConfiguration("enable_sync"),
    }

    if control_mode == "single":
        if single_wrist_side not in ("left", "right"):
            raise RuntimeError("single_wrist_side must be 'left' or 'right'")
        side_prefix = single_wrist_side
        parent_frame = LaunchConfiguration("single_parent_frame").perform(context)
        camera_link = LaunchConfiguration("single_camera_link").perform(context)
        return [
            _static_tf(
                context,
                parent_frame,
                camera_link,
                f"{side_prefix}_camera_xyz",
                f"{side_prefix}_camera_rpy",
            ),
            _realsense_camera(
                rs_launch_path,
                "",
                "wrist_camera",
                LaunchConfiguration("single_serial_no"),
                camera_link,
                **common,
            ),
        ]

    if control_mode == "dual":
        left_parent_frame = LaunchConfiguration("left_parent_frame").perform(context)
        left_camera_link = LaunchConfiguration("left_camera_link").perform(context)
        right_parent_frame = LaunchConfiguration("right_parent_frame").perform(context)
        right_camera_link = LaunchConfiguration("right_camera_link").perform(context)
        return [
            _static_tf(
                context,
                left_parent_frame,
                left_camera_link,
                "left_camera_xyz",
                "left_camera_rpy",
            ),
            _static_tf(
                context,
                right_parent_frame,
                right_camera_link,
                "right_camera_xyz",
                "right_camera_rpy",
            ),
            _realsense_camera(
                rs_launch_path,
                "left",
                "wrist_camera",
                LaunchConfiguration("left_serial_no"),
                left_camera_link,
                **common,
            ),
            _realsense_camera(
                rs_launch_path,
                "right",
                "wrist_camera",
                LaunchConfiguration("right_serial_no"),
                right_camera_link,
                **common,
            ),
        ]

    raise RuntimeError("control_mode must be 'single' or 'dual'")


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory("rm_dualarm")
    defaults = _load_defaults(pkg_share)

    for name, default, description in (
        ("control_mode", defaults["control_mode"], "single | dual"),
        ("single_wrist_side", defaults["single_wrist_side"], "left | right; selects mount transform for single mode"),
        ("single_serial_no", defaults["single_serial_no"], "RealSense serial number for single mode"),
        ("left_serial_no", defaults["left_serial_no"], "RealSense serial number for the left wrist camera"),
        ("right_serial_no", defaults["right_serial_no"], "RealSense serial number for the right wrist camera"),
        ("single_parent_frame", defaults["single_parent_frame"], "Parent frame for the single wrist camera mount"),
        ("single_camera_link", defaults["single_camera_link"], "RealSense root frame for single mode"),
        ("left_parent_frame", defaults["left_parent_frame"], "Parent frame for the left wrist camera mount"),
        ("left_camera_link", defaults["left_camera_link"], "RealSense root frame for the left wrist camera"),
        ("right_parent_frame", defaults["right_parent_frame"], "Parent frame for the right wrist camera mount"),
        ("right_camera_link", defaults["right_camera_link"], "RealSense root frame for the right wrist camera"),
        ("left_camera_xyz", defaults["left_camera_xyz"], "left_Link7 to left_wrist_camera_link xyz"),
        ("left_camera_rpy", defaults["left_camera_rpy"], "left_Link7 to left_wrist_camera_link roll pitch yaw"),
        ("right_camera_xyz", defaults["right_camera_xyz"], "right_Link7 to right_wrist_camera_link xyz"),
        ("right_camera_rpy", defaults["right_camera_rpy"], "right_Link7 to right_wrist_camera_link roll pitch yaw"),
        ("enable_color", defaults["enable_color"], "Enable RealSense color stream"),
        ("enable_depth", defaults["enable_depth"], "Enable RealSense depth stream"),
        ("enable_gyro", defaults["enable_gyro"], "Enable D435i gyro stream"),
        ("enable_accel", defaults["enable_accel"], "Enable D435i accel stream"),
        ("align_depth.enable", defaults["align_depth"], "Enable RealSense align_depth filter"),
        ("pointcloud.enable", defaults["pointcloud"], "Enable RealSense pointcloud filter"),
        ("rgb_camera.color_profile", defaults["color_profile"], "RealSense rgb_camera.color_profile"),
        ("rgb_camera.color_format", defaults["color_format"], "RealSense rgb_camera.color_format"),
        ("rgb_camera.enable_auto_exposure", defaults["color_auto_exposure"], "RealSense rgb_camera.enable_auto_exposure"),
        ("rgb_camera.exposure", defaults["rgb_camera_exposure"], "RealSense rgb_camera.exposure"),
        ("depth_module.depth_profile", defaults["depth_profile"], "RealSense depth_module.depth_profile"),
        ("depth_module.enable_auto_exposure", defaults["depth_auto_exposure"], "RealSense depth_module.enable_auto_exposure"),
        ("gyro_fps", defaults["gyro_fps"], "D435i gyro FPS"),
        ("accel_fps", defaults["accel_fps"], "D435i accel FPS"),
        ("unite_imu_method", defaults["unite_imu_method"], "RealSense unite_imu_method"),
        ("enable_sync", defaults["enable_sync"], "RealSense enable_sync"),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    ld.add_action(OpaqueFunction(function=_setup))
    return ld
