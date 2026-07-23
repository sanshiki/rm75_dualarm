```bash
ros2 launch rm_dualarm real_bringup.launch.py 

ros2 launch rm_dualarm servo_real.launch.py control_mode:=single teleop_type:=vr

# 在docker aubo_ws内
roslaunch ros_tcp_endpoint endpoint.launch

ros2 launch rm_dualarm wrist_cameras.launch.py control_mode:=single

ros2 launch rm_dualarm camera.launch.py
```

VR操作：

| 操作 | 功能 |
|------|------|
| 三击 A | 激活/退出遥操作 |
| 长按 A 3 秒（未激活时） | 回到待机位 |
| 按住 RB | 临时阻塞 VR 目标输出 |
| B | 开始/结束 rosbag 录制 |
| RT | 单臂夹爪开合；双臂右夹爪 |
| LT | 双臂左夹爪 |

bag保存：修改bag名为任务名称。修改地点：`src/rm_dualarm/config/vr_teleop_params.yaml --> record_output_prefix`

** 注：VR需要联网，选择TP-LINK-421即可。PC端有线网开启即可。 **