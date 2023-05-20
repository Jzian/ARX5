#!/usr/bin/env python3
import rospy
from arx5_control.control import RoboticArmAgent
import tkinter as tk
from threading import Thread
import argparse,math
import numpy as np

"""
    这是一个用于进行机械臂实验课程初始校准的程序。
    校准过程需要人工参与，具体步骤参考校准说明。
    命令行参数进行模式配置：
        0：用于基于rviz确定初始位姿对应的关节空间和工作空间的目标值。
"""

# 滑条调参
def parameter_test():
    # 回调函数
    def on_slider_change(val, slider_name):
        global acc_max,vel_max,end_vel_max
        if slider_name == 'Acc_Max':
            acc_max = int(val)/1000
            print('acc_max:',acc_max,'\r\n')
        elif slider_name == 'Vel_Max':
            vel_max = int(val)/1000
            print('vel_max:',vel_max,'\r\n')
        elif slider_name == 'Endvel_Max':
            end_vel_max = int(val)/1000
            print('end_vel_max:',end_vel_max,'\r\n')

    root = tk.Tk()
    root.title("Parameter Tester")
    max_value = 1000
    min_value = 1
    slider_data = [
        {'name': 'Acc_Max', 'init_value': max_value},
        {'name': 'Vel_Max', 'init_value': max_value},
    ]

    for data in slider_data:
        slider_name = data['name']
        init_value = data['init_value']
        # 创建滑动条
        slider = tk.Scale(
            root,
            from_=min_value,
            to=max_value,
            orient=tk.HORIZONTAL,
            command=lambda val, name=slider_name: on_slider_change(val, name),
            label=slider_name,
            length=300
        )
        slider.pack(padx=10, pady=10)
        # 设置滑条初始值
        slider.set(init_value)

    # 创建退出按钮
    exit_button = tk.Button(root, text="Exit", command=root.quit)
    exit_button.pack(padx=10, pady=10)
    root.mainloop()

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Calibration Mode Set")
    parser.add_argument('-m','--mode',type=int,default=0,help='choose the calibration mode, defaut to 0(int),from 0 to 2')
    args, unknown = parser.parse_known_args()

    """系统初始化"""
    # 校准模式（0用于基于rviz确定初始位姿对应的关节空间和工作空间的目标值；1用于运动到确定的目标位置；2用于将给定的pose2joint转换为joint值并打印）
    calibration_mode = 0
    adjust_parameter = False
    joint_to_pose = (np.array([-1,-149,59,1,-1,0])*math.pi/180).tolist()  # moveit的rviz中的目标关节角度（reset前的；reset后会变成当前关节角度，二者是有差别的，可能存在某些电机大于2度的偏差，对末端位姿有较大影响）
    acc_max,vel_max,end_vel_max = 1,0.407,0.5
    # ROS节点初始化
    NODE_NAME = 'arx5_calibration'
    rospy.init_node(NODE_NAME)
    # 是否仿真
    use_sim = True
    control_mode = 'g_i'
    use_gripper = False
    # 初始化机器人实例
    if calibration_mode in [0,2]: init_mode = None
    else: init_mode = 3
    self = RoboticArmAgent(use_sim,control_mode,init_pose=init_mode,node_name=NODE_NAME)
    self.go_to_pick_pose(use_name=True,sleep_time=2)
    if calibration_mode == 2:
        xyz,rpy,_ = self.change_joints_to_pose(joint_to_pose)
        exit(f"转换目标关节值得到目标姿态值为：xyz={xyz},rpy={rpy}")
    # 打印初始状态信息
    self.log_arm_info(current_pose=True,current_joint=True)  # 确定关节空间和工作空间的绝对对应关系

    """ 键盘控制按步骤完成校准 """
    if adjust_parameter: Thread(target=parameter_test,daemon=True).start()
    pick_cnt = place_cnt = 0
    while self.is_alive():
        key = self.key_get(0)
        if key == '0':  # 移动到Home(退出校准时用)
            self.go_to_named_or_joint_target('Home',sleep_time=2,return_enable=True,start_base=1)
        elif key == '1':  # 移动到pick初始态
            self.go_to_pick_pose(use_name=True,sleep_time=1,from_place=False)
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
            self.get_current_state()
            print("当前关节角度：",tuple(self.current_joints))
            print("xyz:",tuple(self.current_xyz),"rpy:",tuple(self.current_rpy))
        elif key == '\b':
            print(f'检测到按下{key}键')
        else:  # 键盘控制
            print("acc_max=",acc_max,"vel_max=",vel_max,"end_vel_max=",end_vel_max)
            # self.arm_moveit.limit_max_cartesian_link_speed(end_vel_max,link_name=self.end_effector_link)
            self.key_control(delta_task=0.001,delta_joint=0.0174,key=key,acc_limit=acc_max,vel_limit=vel_max)