# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于瑞尔曼 (RealMan) 机械臂的双臂操作具身智能平台。使用 ROS 2 Humble 和 MoveIt2 进行运动规划，支持 Gazebo 仿真和真实硬件。

## 常用命令

### 构建

```bash
# 从工作空间根目录构建所有包
colcon build

# 构建特定包
colcon build --packages-select <package_name>

# 构建时启用符号链接安装（开发时推荐，避免重复 install）
colcon build --symlink-install

# 清理构建
rm -rf build/ install/ log/
colcon build
```

### 运行前

```bash
source install/setup.bash
```

### 启动（RM75B 是主要目标机型）

```bash
# Gazebo 仿真
ros2 launch rm_gazebo gazebo_75_demo.launch.py

# MoveIt2 + Gazebo 联合仿真
ros2 launch rm_75_config gazebo_moveit_demo.launch.py

# 纯 MoveIt2 demo（无 Gazebo）
ros2 launch rm_75_config demo.launch.py

# 真实机器人 MoveIt2
ros2 launch rm_75_config real_moveit_demo.launch.py

# 真实机器人驱动节点（TCP 通信）
ros2 launch rm_driver rm_75_driver.launch.py
```

### 调试

```bash
# 查看活跃的 ROS topic
ros2 topic list

# 回显特定 topic
ros2 topic echo /rm_driver/arm_state

# 查看节点图
rqt_graph
```

## 架构

### 包依赖层级

```
rm_ros_interfaces  (自定义 msg，全部为 .msg 文件，无 srv/action)
       ↑
  rm_driver         (硬件驱动，TCP socket 与机械臂通信)
  rm_control        (控制算法，含 cubicSpline 插值)
       ↑
  rm_example        (API 示例代码)
  rm_arm_examples   (进阶示例：力控、轨迹运动)
       ↑
  rm_bringup        (组合 launch 文件，桥接不同系统)
  rm_gazebo         (Gazebo 仿真 launch)
  rm_description    (URDF 模型、mesh 文件)
  rm_moveit2_config/* (MoveIt2 运动规划配置，每种机型一个子包)
```

### 核心包说明

- **rm_ros_interfaces**: ~70+ 自定义 ROS 2 消息类型。涵盖机械臂状态 (`Armstate`)、关节位置 (`Jointpos`)、笛卡尔位姿 (`Cartepos`)、运动命令 (`Movej`/`Movel`/`Movec`/`Movejp`)、力控 (`Forcepositionmove`)、夹爪 (`Gripperset`/`Gripperpick`)、灵巧手 (`Handangle`/`Handforce`/`Handspeed`/`Handposture`)、Modbus 通信等。所有其他包均依赖此包定义的接口。

- **rm_driver**: 通过 TCP socket（非阻塞模式）与真实机械臂通信。提供运动控制、状态查询、力控、夹爪/手部控制等全部 API。作为 ROS 2 Node 运行，订阅自定义 motion command topic 并发布 arm state topic。IP 和端口通过 launch 参数配置。

- **rm_control**: 独立的控制节点，包含三次样条插值（`cubicSpline.h`），用于轨迹平滑。订阅/发布 `rm_ros_interfaces` 定义的 topic。

- **rm_moveit2_config/rm_XX_config**: 每种机械臂型号（63/65/75/eco63/eco65/gen72）及其子变体（标准/6f/6fb/III）有独立的 MoveIt2 配置包。每个配置包结构相同：`move_group`、`rsp`（robot state publisher）、`spawn_controllers`、`moveit_rviz`、以及模拟/真实/ Gazebo 联合等 demo launch 文件。

### 机械臂型号命名规则

- 基础型号：`rm_63`, `rm_65`, `rm_75`, `eco63`, `eco65`, `gen72`
- 子变体后缀：
  - `_6f` = 六维力传感器版本
  - `_6fb` = 六维力传感器 + 基座力传感器版本
  - `_III` = 第三代
  - 无后缀 = 标准版

### Launch 文件组织

Launch 文件按三组路径组织：
1. `rm_gazebo/launch/` — 纯仿真环境
2. `rm_driver/launch/` / `rm_control/launch/` — 真实机器人驱动/控制
3. `rm_bringup/launch/` — 组合 launch，串联多个子 launch（如 Gazebo + MoveIt2）
4. `rm_*_config/launch/` — MoveIt2 相关（每个机器人型号独立配置）

### 源代码

所有源码位于 `src/ros2_rm_robot/`，该目录是 git submodule，上游为 `https://github.com/RealManRobot/ros2_rm_robot.git`。

- C++ 包使用 `ament_cmake` 构建，依赖 `rclcpp`
- Python launch 文件使用 `ros2 launch` 系统
- 自定义消息包属于 `rosidl_interface_packages` 组
