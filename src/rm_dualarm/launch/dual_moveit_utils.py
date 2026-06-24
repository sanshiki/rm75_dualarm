import copy
import os
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


JOINTS = [f"joint{i}" for i in range(1, 8)]
LINKS = ["base_link"] + [f"Link{i}" for i in range(1, 8)]


def _prefix_robot(root, prefix, base_xyz):
    for elem in root.iter():
        for attr in ("name", "link", "reference"):
            if attr in elem.attrib and elem.attrib[attr] in LINKS + JOINTS:
                elem.attrib[attr] = prefix + elem.attrib[attr]
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None and parent.attrib.get("link") in LINKS:
            parent.attrib["link"] = prefix + parent.attrib["link"]
        if child is not None and child.attrib.get("link") in LINKS:
            child.attrib["link"] = prefix + child.attrib["link"]
    for ros2_control in root.findall("ros2_control"):
        ros2_control.attrib["name"] = f"{prefix}GazeboSystem"
        for joint in ros2_control.findall("joint"):
            if joint.attrib.get("name") in JOINTS:
                joint.attrib["name"] = prefix + joint.attrib["name"]

    fixed = ET.Element("joint", {"name": f"{prefix}world_fixed", "type": "fixed"})
    ET.SubElement(fixed, "origin", {"xyz": base_xyz, "rpy": "0 0 0"})
    ET.SubElement(fixed, "parent", {"link": "world"})
    ET.SubElement(fixed, "child", {"link": f"{prefix}base_link"})
    return fixed


def _remove_world_and_plugin(root):
    for child in list(root):
        if child.tag == "link" and child.attrib.get("name") == "world":
            root.remove(child)
        if child.tag == "joint" and child.attrib.get("name") == "fixed":
            root.remove(child)
        if child.tag == "gazebo" and child.find("plugin") is not None:
            root.remove(child)


def load_dual_arm_layout(pkg_share):
    path = os.path.join(pkg_share, "config", "dual_arm_layout.yaml")
    defaults = {"base_x": 0.0, "base_z": 0.0, "base_spacing_y": 0.90}
    if not os.path.exists(path):
        return defaults
    with open(path, "r", encoding="utf-8") as f:
        loaded = yaml.safe_load(f) or {}
    defaults.update({k: float(v) for k, v in loaded.items() if k in defaults})
    return defaults


def resolve_dual_arm_layout(pkg_share, overrides=None):
    layout = load_dual_arm_layout(pkg_share)
    for key, value in (overrides or {}).items():
        if value is None or value == "":
            continue
        layout[key] = float(value)
    return layout


def build_dual_robot_description(pkg_share, layout=None):
    layout = layout or load_dual_arm_layout(pkg_share)
    try:
        rm_gazebo_share = get_package_share_directory("rm_gazebo")
    except Exception:
        rm_gazebo_share = os.path.abspath(
            os.path.join(pkg_share, "..", "ros2_rm_robot", "rm_gazebo"))
    urdf_path = os.path.join(rm_gazebo_share, "config", "gazebo_75_description.urdf.xacro")
    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    source = ET.fromstring(doc.toxml())
    _remove_world_and_plugin(source)

    robot = ET.Element("robot", {"name": "rm_75_dualarm"})
    ET.SubElement(robot, "link", {"name": "world"})

    left = copy.deepcopy(source)
    right = copy.deepcopy(source)
    base_x = float(layout.get("base_x", 0.0))
    base_z = float(layout.get("base_z", 0.0))
    half_y = float(layout.get("base_spacing_y", 0.90)) / 2.0
    robot.append(_prefix_robot(left, "left_", f"{base_x} {half_y} {base_z}"))
    robot.append(_prefix_robot(right, "right_", f"{base_x} {-half_y} {base_z}"))
    for child in list(left):
        robot.append(child)
    for child in list(right):
        robot.append(child)

    gazebo = ET.SubElement(robot, "gazebo")
    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {"filename": "libgazebo_ros2_control.so", "name": "gazebo_ros2_control"},
    )
    ET.SubElement(plugin, "parameters").text = os.path.join(
        pkg_share, "config", "dual_ros2_controllers.yaml")
    ET.SubElement(plugin, "robot_param").text = "robot_description"
    ET.SubElement(plugin, "robot_param_node").text = "robot_state_publisher"

    return ET.tostring(robot, encoding="unicode")


def _prefix_link_name(name, prefix):
    if name in LINKS:
        return prefix + name
    return name


def _build_dual_srdf(config):
    source_srdf = ET.fromstring(config["robot_description_semantic"])
    robot = ET.Element("robot", {"name": "rm_75_dualarm"})

    left_group = ET.SubElement(robot, "group", {"name": "left_rm_group"})
    ET.SubElement(left_group, "chain", {
        "base_link": "left_base_link",
        "tip_link": "left_Link7",
    })
    right_group = ET.SubElement(robot, "group", {"name": "right_rm_group"})
    ET.SubElement(right_group, "chain", {
        "base_link": "right_base_link",
        "tip_link": "right_Link7",
    })

    for elem in source_srdf.findall("disable_collisions"):
        for prefix in ("left_", "right_"):
            ET.SubElement(robot, "disable_collisions", {
                "link1": _prefix_link_name(elem.attrib["link1"], prefix),
                "link2": _prefix_link_name(elem.attrib["link2"], prefix),
                "reason": elem.attrib.get("reason", "Never"),
            })

    return ET.tostring(robot, encoding="unicode")


def dual_moveit_config(pkg_share, layout=None):
    robot_description = build_dual_robot_description(pkg_share, layout)
    rm75_config = MoveItConfigsBuilder(
        "rm_75_description", package_name="rm_75_config").to_moveit_configs()
    config = rm75_config.to_dict()
    srdf = _build_dual_srdf(config)
    kinematics = config.get("robot_description_kinematics", {})
    if "rm_group" in kinematics:
        kinematics["left_rm_group"] = dict(kinematics["rm_group"])
        kinematics["right_rm_group"] = dict(kinematics["rm_group"])
        kinematics.pop("rm_group", None)
    moveit_controllers = {
        "controller_names": [
            "left_rm_group_controller",
            "right_rm_group_controller",
        ],
        "left_rm_group_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [f"left_joint{i}" for i in range(1, 8)],
        },
        "right_rm_group_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [f"right_joint{i}" for i in range(1, 8)],
        },
    }
    config["robot_description"] = robot_description
    config["robot_description_semantic"] = srdf
    config["robot_description_kinematics"] = kinematics
    config["moveit_simple_controller_manager"] = moveit_controllers
    return config
