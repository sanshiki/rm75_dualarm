目前单臂的vr/mouse仿真遥操已经打通.现在需要构建双臂的仿真遥操,从target_pose发布端到gazebo仿真端的完整pipeline都需要双臂的适配.

期望:
 - 可以使用vr/mouse进行双臂仿真遥操.鼠标仅用于测试,最终目的是使用vr进行双臂遥操.
 - 留出接口给夹爪信号,但是先不用实现.

必要信息:
/quest/joystick
 - buttons: X A B Y  0 0 0 0 LT RT LB LB LS RS
 - axies: right_bar_x, right_bar_y, left_bar_x, left_bar_y, LT, RT, LB, RB

摇杆坐标系：右为X正方向，上为Y正方向