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

`rm_dualarm` 是基于 MoveIt2 Servo 的动态伺服控制包，支持鼠标遥操作实时控制机械臂末端位姿。

### 启动

```bash
# 仿真 — 两个终端
ros2 launch rm_dualarm sim_bringup.launch.py          # T1: Gazebo
ros2 launch rm_dualarm servo_sim.launch.py               # T2: Servo + 鼠标遥操作

# 真机 — 两个终端
ros2 launch rm_driver rm_75_driver.launch.py             # T1: 驱动
ros2 launch rm_dualarm servo_real.launch.py              # T2: Servo
```

### 参数

```bash
# 调整灵敏度
ros2 launch rm_dualarm servo_sim.launch.py linear_scale:=0.002 angular_scale:=0.01

# 使用 twist 模式（joystick / 其他速度源）
ros2 launch rm_dualarm servo_sim.launch.py use_pose_tracking:=false
```

### 数据流

```
Sim:  mouse_teleop → /target_pose (PoseStamped) → servo_pose_tracking_demo
      → /rm_group_controller/joint_trajectory → Gazebo

Real: mouse_teleop → /target_pose → servo_pose_tracking_demo
      → servo_bridge (50Hz) → /rm_driver/movej_canfd_cmd → 机械臂
```