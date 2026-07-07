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
ros2 launch rm_dualarm real_bringup.launch.py arm_ip:=192.168.1.18
 ```

 - 相机
```bash
ros2 launch rm_dualarm camera.launch.py 
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

双臂仿真使用左右命名空间：

```
/left/target_pose  → left_pose_tracking  → /left_servo_node/delta_twist_cmds
  → /left_rm_group_controller/joint_trajectory

/right/target_pose → right_pose_tracking → /right_servo_node/delta_twist_cmds
  → /right_rm_group_controller/joint_trajectory
```

双臂真机数据流：

```
teleop → /left/target_pose, /right/target_pose
  → left/right_pose_tracking → left/right_servo_node
  → left/right_servo_bridge → /left/rm_driver/movej_canfd_cmd
                             → /right/rm_driver/movej_canfd_cmd
  → 左臂 (left_arm_ip) + 右臂 (right_arm_ip)

反馈:
  /left/joint_states + /right/joint_states
  → dual_joint_state_merger → /joint_states (left_joint1..7, right_joint1..7)
  → robot_state_publisher → TF (left_*, right_* frames)
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
| dual_joint_state_merger | `scripts/dual_joint_state_merger.py` | 合并左右臂 joint_states 并加前缀 |

### 启动

```bash
# === 仿真 ===
# T1: Gazebo + move_group + pose_init（臂到非奇异位姿）
ros2 launch rm_dualarm sim_bringup.launch.py
# T2: Servo + 遥操作
ros2 launch rm_dualarm servo_sim.launch.py teleop_type:=mouse
```

### 双臂仿真测试

先准备环境：

```bash
conda activate rm75
source install/setup.bash
```

启动 Gazebo 双臂模型和控制器：

```bash
# T1: 双臂 Gazebo + robot_state_publisher + move_group + 双臂待机位初始化
ros2 launch rm_dualarm dual_sim_bringup.launch.py
```

`dual_sim_bringup.launch.py` 默认会在左右 Gazebo 控制器加载完成后运行 `pose_init.py control_mode:=dual`，让 `left_rm_group` 和 `right_rm_group` 依次从 0 位移动到待机姿态。待机关节从 `config/standby_pose.yaml` 读取；可用 `standby_pose_file:=/path/to/standby_pose.yaml` 覆盖。也可用 `use_pose_init:=false` 跳过，或用 `init_delay:=10.0` 调整等待时间：

```bash
ros2 launch rm_dualarm dual_sim_bringup.launch.py use_pose_init:=false
ros2 launch rm_dualarm dual_sim_bringup.launch.py init_delay:=10.0
```

双臂仿真间距默认从 `config/dual_arm_layout.yaml` 读取。修改 `base_spacing_y` 后，Gazebo bringup 和 Servo 都会使用同一份双臂模型；需要重启 Gazebo 和 Servo 两个 launch 才会同时生效。临时调试 Gazebo 间距也可用：

```bash
ros2 launch rm_dualarm dual_sim_bringup.launch.py base_spacing_y:=0.72
```

注意：如果临时用 launch 参数覆盖 Gazebo 间距，Servo/RViz 仍会读取 `dual_arm_layout.yaml`，除非也同步修改该 YAML。正式测试建议只改 YAML，避免 TF 和 Gazebo 模型不一致。

另开终端启动双臂 Servo。鼠标测试用 `teleop_type:=mouse`，VR 测试用 `teleop_type:=vr`：

```bash
# T2: 双臂 Servo + 鼠标遥操作
ros2 launch rm_dualarm servo_sim.launch.py control_mode:=dual teleop_type:=mouse

# 或：双臂 Servo + VR 遥操作
ros2 launch rm_dualarm servo_sim.launch.py control_mode:=dual teleop_type:=vr
```

鼠标测试时，按住左键开始发布左右目标；松开左键停止。VR 测试时，三击 A 激活/取消，按住 RB 阻塞输出，B 切换 normal/incremental 模式。VR 输入来自 `/quest/joystick` 和 TF `hand_left`、`hand_right`；不连接 TCP relay 做本地测试时可用：

```bash
ros2 launch rm_dualarm vr_teleop.launch.py control_mode:=dual use_relay_receiver:=false
```

VR 标定参数模板在 `config/vr_calibration.yaml`。实际测试建议为每个操作者生成一份标定文件：先确认双臂已经由 `pose_init` 到达 `standby_pose.yaml` 定义的待机动作；操作者直立，双手自然握遥控器，大臂紧贴身体下垂，小臂 90 度平行地面抬起，保持遥控器和地面平行，然后按 `Y` 采样。脚本会记录该预设动作下的头显高度、左右手位姿和左右机械臂末端待机位姿，并生成 `schema_version: 2` 标定文件。

```bash
ros2 launch rm_dualarm vr_calibration.launch.py output_file:=/tmp/vr_calibration.yaml
# 保持预设动作，按 Y；默认采样 2 秒求均值
ros2 launch rm_dualarm servo_sim.launch.py control_mode:=dual teleop_type:=vr \
  vr_calibration_file:=/tmp/vr_calibration.yaml
```

新版标定会把预设动作绑定到机械臂待机末端位姿，之后按手部相对预设动作的增量控制目标。若要调整灵敏度，修改生成文件中的 `left/right.position_scale`；若某套 VR 坐标的右手 Y 方向相反，可把 `right.position_scale` 改成 `[1.0, -1.0, 1.0]`；若要减弱姿态跟随，修改 `left/right.rotation_scale`。

链路验证命令：

```bash
# target_pose 是否由遥操作端发出
ros2 topic echo /left/target_pose --once
ros2 topic echo /right/target_pose --once

# Servo 是否向 Gazebo 控制器输出轨迹
ros2 topic echo /left_rm_group_controller/joint_trajectory --once
ros2 topic echo /right_rm_group_controller/joint_trajectory --once

# VR 夹爪占位接口：LT→left，RT→right
ros2 topic echo /left/gripper_cmd --once
ros2 topic echo /right/gripper_cmd --once
```

无 GUI 服务器上测试可关闭 Gazebo 界面：

```bash
ros2 launch rm_dualarm dual_sim_bringup.launch.py gazebo_gui:=false
```

### 真机与 VR Relay

```bash
# === 单臂真机 ===
# T1: 真机 bringup (rm_driver + rm_control + move_group + 可选 pose_init)
ros2 launch rm_dualarm real_bringup.launch.py arm_ip:=192.168.1.18
# T2: Servo + 遥操作
ros2 launch rm_dualarm servo_real.launch.py control_mode:=single teleop_type:=mouse

# T1 (可选): 带待机位初始化
ros2 launch rm_dualarm real_bringup.launch.py arm_ip:=192.168.1.18 use_pose_init:=true

# === 双臂真机 ===
# T1: 双臂 bringup (两个 rm_driver + joint_state_merger + move_group)
ros2 launch rm_dualarm dual_real_bringup.launch.py \
    left_arm_ip:=192.168.1.18 right_arm_ip:=192.168.1.19
# T2: 双臂 Servo + 遥操作
ros2 launch rm_dualarm servo_real.launch.py control_mode:=dual teleop_type:=mouse

# 双臂带待机位初始化
ros2 launch rm_dualarm dual_real_bringup.launch.py \
    left_arm_ip:=192.168.1.18 right_arm_ip:=192.168.1.19 use_pose_init:=true

# 双臂间距覆盖 (物理底座间距)
ros2 launch rm_dualarm dual_real_bringup.launch.py base_spacing_y:=0.90
ros2 launch rm_dualarm servo_real.launch.py control_mode:=dual base_spacing_y:=0.90

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

姿态跟踪模式由 `config/pose_tracking_settings.yaml` 的 `orientation_tracking_mode` 控制：

```yaml
orientation_tracking_mode: full_quat # full_quat | yaw_only
```

### 数据采集与离线分析

录制双臂遥操作数据：

```bash
ros2 launch rm_dualarm record_dual_teleop.launch.py output:=bags/dual_test_001
```

离线生成频率统计和轨迹图：

```bash
ros2 run rm_dualarm analyze_dual_teleop_bag.py bags/dual_test_001 \
  --output analysis/dual_test_001
```

输出包括 `summary.md`、左右 joint_state vs trajectory command 曲线、左右 target pose 曲线。

### 可视化

| Topic | 类型 | 内容 |
|------|------|------|
| `/target_pose` | PoseStamped | 当前追踪目标 |
| `/pose_tracking/safe_zone` | Marker | 安全区线框 (RViz) |
| `/servo_node/status` | Int8 | Servo 状态码 |
