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
```

 - moveit启动
 ```bash
ros2 launch rm_75_config gazebo_moveit_demo.launch.py
```