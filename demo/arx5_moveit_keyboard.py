#!/usr/bin/env python3
import rospy
from arx5_control.control import RoboticArmAgent

# ROS节点初始化
NODE_NAME = 'arm_key'
rospy.init_node(NODE_NAME)
# 初始化机器人实例
arm = RoboticArmAgent(use_sim=True,node_name=NODE_NAME)
print('现在可以用键盘控制移动，尝试按下：wasd ijkl观察机械臂运动，按下t可以实现工作空间的控制和关节空间的控制的相互转换')
while arm.is_alive():
    key = arm.key_get(0)
    if key == 'q':
        arm.go_to_named_or_joint_target({0:3}) 
        height = 0
        place_z = arm._place_base_z + height
        arm.set_and_go_to_pose_target([*arm._place_xy,place_z],arm._place_rpy,sleep_time=1)        
    elif key == 'w':
        success = arm.set_and_go_to_pose_target(arm._pick_scan_xyz_dict[0],arm._pick_rpy,
                                                sleep_time=2,return_enable=True)
        if not success:
            arm.go_to_named_or_joint_target('PickSim')
            success = arm.set_and_go_to_pose_target(arm._pick_scan_xyz_dict[0],arm._pick_rpy,
                                                sleep_time=2)
