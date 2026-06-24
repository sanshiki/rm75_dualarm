# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 Humble workspace for a RealMan RM75B dual-arm manipulation platform. Local development is centered in `src/rm_dualarm/`: nodes in `src/`, scripts in `scripts/`, launch files in `launch/`, parameters in `config/`, and RViz setup in `rviz/`. Upstream RealMan packages live in `src/ros2_rm_robot/` as a git submodule; treat them as vendor code unless a change is explicit.

## Architecture Overview

The main Servo path is:

```text
teleop (mouse/VR) -> /target_pose -> pose_tracking_node -> servo_node_main
  -> /rm_group_controller/joint_trajectory -> Gazebo
  or servo_bridge -> /rm_driver/movej_canfd_cmd -> real robot
```

For dual-arm simulation, use `control_mode:=dual`; targets and controllers are left/right namespaced where supported.

## Build, Test, and Development Commands

- `conda env create -f rm75_env.yaml`: create the Python/ROS environment.
- `conda activate rm75`: activate the workspace environment.
- `colcon build --symlink-install`: build all packages for development.
- `colcon build --packages-select rm_dualarm --symlink-install`: rebuild only `rm_dualarm`.
- `source install/setup.bash`: load the workspace overlay after each build.
- `ros2 launch rm_dualarm sim_bringup.launch.py`: start simulation bringup.
- `ros2 launch rm_dualarm servo_sim.launch.py teleop_type:=mouse`: run simulated Servo teleop.
- `ros2 launch rm_dualarm servo_real.launch.py teleop_type:=mouse`: run Servo against hardware.
- `ros2 launch rm_bringup rm_75_bringup.launch.py`: start the RM75 real-robot bringup.

## Coding Style & Naming Conventions

Follow existing ROS 2 conventions. C++ uses C++14 with `-Wall -Wextra -Wpedantic`; keep C++ sources in `src/` and headers under `include/rm_dualarm/`. Python nodes use `rclpy`, executable scripts, snake_case filenames, and `_node.py` suffixes. Launch files end in `.launch.py`; parameters belong in YAML files under `config/`.

## Testing Guidelines

Run `colcon test --packages-select rm_dualarm`, then inspect with `colcon test-result --verbose`. For runtime validation, use `ros2 topic list`, `ros2 node list`, `rqt_graph`, `ros2 topic echo /target_pose --once`, `ros2 topic hz /target_pose`, and controller topics such as `/rm_group_controller/joint_trajectory` or left/right equivalents.

## Commit & Pull Request Guidelines

Recent commits use short imperative summaries, for example `add pose_tracking_node` and `update claude and readme`. Keep commits focused on one behavior or package area. Pull requests should describe the affected mode (`sim`, `real`, `mouse`, `VR`, or `dual`), list validation commands, link issues, and include logs or screenshots for Gazebo, RViz, or hardware-facing changes.

## Safety & Configuration Tips

Do not hard-code robot IPs, gains, workspace bounds, or Servo limits when YAML parameters exist. Review `config/pose_tracking_settings.yaml`, `config/servo_75_sim.yaml`, `config/servo_75_real.yaml`, `config/mouse_teleop_params.yaml`, and `config/vr_teleop_params.yaml` before behavior changes. For real robots, start the driver first and verify `/joint_states` plus `/rm_driver/arm_state`.
