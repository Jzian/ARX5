#!/usr/bin/env python3

import rospy
from arx5_control.control import RoboticArmAgent
import argparse

"""
    这是一个用于进行机械臂实验课程实机初始校准的程序。
    校准过程需要人工参与，具体步骤参考校准说明。
    启动程序后，机械臂首先移动到初始位置(η)。
    命令行参数进行模式配置：
        0：初始高度值校准：
            将机械臂的pick位置的Z轴高度调整为下方PICK_POSE_Z给出的值，
            脚本启动后机械臂将自动移动到该新位置，
            然后在该位置基础上通过按键进行微调(包括微调rpy值)，
            位置合适后按=键，将打印出目标Z轴值和关节角度值。建议按3次=键，观察目标关节角度值是否具有一致性。
            然后将这些值在代码中进行修改，即可完成初始高度值的校准。
        1：视觉阈值整定：
            高度校准完成后，运行该程序，机械臂将运动到待识别位置处。然后启动视觉参数整定的程序。
            整定参数启动的同时会在终端打印出初始的阈值，可以据此为基准进行细化调整以适应当前环境。
        2：视觉基准调整：
            执行后机械臂自动移动到识别位置。然后在终端中输入rosrun arx5_vison Best_Contour.py -t pi0/pi1进行识别效果检查。
            若效果OK，可以根据提示的偏差信息在Best_Contour.py中修改。
            调整好物块位置后，可以通过键盘2、3、4键将第一个物块抓起抬高，然后将第二个物块放到第一个物块下方。
            完成后按8释放夹爪。按0回到Home位置。
"""

PICK_POSE_Z = -0.065  # 粗位置，后续用键盘微调

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Calibration Mode Set")
    parser.add_argument('-m','--mode',type=int,default=1,help='choose the calibration mode, defaut to 0(int),from 0 to 2')
    args, unknown = parser.parse_known_args()

    NODE_NAME = 'arx5_calibration'
    rospy.init_node(NODE_NAME)
    self = RoboticArmAgent(False,node_name=NODE_NAME)
    self.task_pick_place_param_init(1)  # 在其中修改校准值
    # self.go_to_named_or_joint_target('PickReal',sleep_time=2)
    if args.mode == 0:
        self._pick_scan_xyz_dict[0][2] = PICK_POSE_Z
        self._pick_scan_xyz_dict[1][2] = PICK_POSE_Z
        self._pick_base_z = self._place_base_z = PICK_POSE_Z
        self.go_to_pick_pose(use_name=False,sleep_time=1)
    elif args.mode == 1:
        self.go_to_pick_pose(use_name=True,sleep_time=2)
        # self.go_to_named_or_joint_target('PickUnder')
        rospy.set_param("/vision_attention",'HSV')  # HSV|YCrCb|Lab,推荐HSV
    elif args.mode == 2:
        self.go_to_pick_pose(use_name=True,sleep_time=2)

    """ 键盘控制按步骤完成校准 """
    pick_cnt = place_cnt = 0
    while self.is_alive():
        key = self.key_get(0)
        if key == '1':  # 移动到pick初始态
            if args.mode == 0: use_name = False
            else: use_name = True
            self.go_to_pick_pose(use_name,sleep_time=1,from_place=False)
        elif key == '2':  # 下降到待抓取位置
            self.go_to_single_axis_target(2,self._pick_base_z,target_ref='l',sleep_time=1,vel_limit=0.407)  # 首先到达可抓取的高度位置(z单轴移动)
        elif key == '3':  # 抓取
            self.gripper_control(1)
        elif key == '4':  # 抬高一个物块+1mm的距离(可用于确定place阶段的参考上边沿)
            self.go_to_single_axis_target(2,self._pick_base_z+0.026,vel_limit=0.407,sleep_time=1)
        elif key == '5':  # 视觉识别切换
            rospy.set_param("/vision_attention",f'pick{pick_cnt}')
            pick_cnt ^= 1
        elif key == '6':  # 视觉识别切换
            rospy.set_param("/vision_attention",f'place{place_cnt}')
            place_cnt ^= 1
        elif key == '7':  # 颜色阈值确定
            rospy.set_param("/vision_attention",'HSV')  # HSV|YCrCb|Lab,推荐HSV
        elif key == '8':  # 释放
            self.gripper_control(0)
        elif key == '9':  # 到达首次place位置（用来测试是否桌面倾斜，一般离地高度1mm左右可接受，若出现与底面碰撞一般可忽略）
            self.go_to_place_pose(first=True)  # 第一次特殊，不经过高度测量以及调节，直接到达待放置位置
        elif key == '-':  # 到达第二次place的位置
            self.go_to_place_pose(use_tof=self.use_tof,place_mode=1)
        elif key == '=':
            if args.mode == 0:
                print(f'目标Z轴高度z={self.last_xyz[2]}')
                print(f'目标关节角度值={self.change_pose_to_joints_value(self.last_target_pose)}')
                print(f'目标rpy=：{self.current_rpy}')
            else: self.log_arm_info(target_joint=True,target_pose=True,current_joint=True,current_pose=True)
        elif key == '\b':
            print(f'检测到按下退格键，无事发生......')
        else:  # 键盘控制
            acc_max,vel_max,end_vel_max = 1,0.407,0.5
            self.key_control(delta_task=0.001,delta_joint=0.0174,key=key,acc_limit=acc_max,vel_limit=vel_max)