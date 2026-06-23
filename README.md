# 瑞尔曼双臂平台

## 简介

基于瑞尔曼RM75B机械臂的双臂操作具身智能平台

## 安装

### 安装rm driver
参考：`https://develop.realman-robotics.com/robot/ros2/getStarted/`

### 安装conda环境
```bash
conda env create -f rm75_env.yaml
```

## 启动

 - 基本环境
```bash
source install/setup.bash
conda activate rm75
```

 - gazebo仿真
```bash
ros2 launch rm_gazebo gazebo_75_demo.launch.py
ros2 launch rm_75_config gazebo_moveit_demo.launch.py
```

 - 真机
 ```bash
ros2 launch rm_bringup rm_75_bringup.launch.py
 ```

## rm_dualarm — 双臂伺服与遥操作

基于 MoveIt2 Servo 的动态伺服控制包，支持鼠标/VR 遥操作实时控制机械臂末端位姿。

### 架构

```
teleop (mouse/VR) → /target_pose (PoseStamped)
  → pose_tracking_node (PID + 安全区 + 低通滤波 → TwistStamped)
  → servo_node_main (Jacobian 持续伺服)
  → /rm_group_controller/joint_trajectory → Gazebo (sim)
  或 servo_bridge → /rm_driver/movej_canfd_cmd → 机械臂 (real)
```

### 核心模块

| 模块 | 文件 | 功能 |
|------|------|------|
| mouse_teleop | `src/mouse_teleop_node.py` | 屏幕绝对位置→YZ平面映射，左键拖动控制 |
| vr_teleop | `src/vr_teleop_node.py` | VR 手部追踪，支持 normal/incremental 模式 |
| pose_tracking | `src/pose_tracking_node.py` | PID 位姿跟踪 + 安全区 clamp + 低通滤波 |
| servo_bridge | `src/servo_bridge.cpp` | Servo JointTrajectory → rm_driver CANFD (50Hz) |
| pose_init | `scripts/pose_init.py` | 初始化机械臂到非奇异位姿 (move_group 规划) |
| trajectory_relay | `scripts/trajectory_relay.py` | 门控转发，遥操激活后才开放 |

### 启动

```bash
# === 仿真 ===
# T1: Gazebo + move_group + pose_init（臂到非奇异位姿）
ros2 launch rm_dualarm sim_bringup.launch.py
# T2: Servo + 遥操作
ros2 launch rm_dualarm servo_sim.launch.py teleop_type:=mouse

# === 真机 ===
# T1: 驱动
ros2 launch rm_driver rm_75_driver.launch.py
# T2: Servo + 遥操作
ros2 launch rm_dualarm servo_real.launch.py teleop_type:=mouse

# === VR 遥操作 ===
# T1 (宿主机): 启动 relay_receiver + broadcaster + vr_teleop
ros2 launch rm_dualarm vr_teleop.launch.py
# T2 (Docker aubo-ros): 启动 ROS 1 endpoint + relay_sender
docker exec aubo-ros bash -c "source aubo_entry.sh && \
  roslaunch ros_tcp_endpoint endpoint.launch & \
  python3 /tmp/relay_sender.py --host <宿主机IP> --port 7654"
```

### 遥操作控制

| 操作 | 鼠标 | VR |
|------|------|-----|
| 激活 | 按住左键 | 三击 A |
| 平移 Y/Z | 移动鼠标 | normal 模式手部位移 |
| 平移 X | 滚轮 | — |
| 旋转 | — | incremental 模式 |
| 停止 | 松开左键 | 三击 A 取消 / 按住 RB |
| 模式切换 | — | 按 B (normal↔incremental) |

### 配置参数

```bash
# 鼠标灵敏度（工作平面范围）
ros2 launch rm_dualarm servo_sim.launch.py fixed_x:=0.3 y_min:=-0.3 y_max:=0.3

# VR 灵敏度
ros2 launch rm_dualarm vr_teleop.launch.py p_sensitivity:=2.0 q_sensitivity:=2.0

# PID / 滤波 / 安全区 → 修改 config/pose_tracking_settings.yaml
# 伺服速度 / 奇异性 → 修改 config/servo_75_sim.yaml
```

### 可视化

| Topic | 类型 | 内容 |
|------|------|------|
| `/target_pose` | PoseStamped | 当前追踪目标 |
| `/pose_tracking/safe_zone` | Marker | 安全区线框 (RViz) |
| `/servo_node/status` | Int8 | Servo 状态码 |