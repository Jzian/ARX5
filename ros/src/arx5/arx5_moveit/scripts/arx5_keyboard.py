#!/usr/bin/env python3
import rospy
from arx5_control.control import RoboticArmAgent
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser("param set")
    parser.add_argument('-s','--use_sim',action='store_true',help='use sim or real robot')  # 默认针对实机
    args, unknown = parser.parse_known_args()  # 这一步本质上是根据设定的变量从sys.argv中对已知和未知的参数进行分类，以及参数和对应值的提取
    """系统初始化"""
    acc_max,vel_max = 1,1
    # ROS节点初始化
    NODE_NAME = 'arx5_moveit_keyboard'
    rospy.init_node(NODE_NAME)
    # 是否仿真
    self = RoboticArmAgent(args.use_sim,init_pose=None,node_name=NODE_NAME)
    # if not args.use_sim:
    #     self.go_to_named_or_joint_target('PickUnder',sleep_time=2)
    # else: self.go_to_named_or_joint_target('PickSim',sleep_time=2)
    """ 键盘控制 """
    print('可以使用键盘控制，按键表如下（初始为工作空间）：')
    print('  工作空间控制：(按t键切换为关节空间)\r\n  w/s:前进/后退\r\n  a/d:左移/右移\r\n  q/e:上升/下降\r\n  j/l:yaw左/右\r\n  i/k:pitch上/下\r\n  u/o:roll逆/顺')
    print('  关节空间控制：(按t键切换为工作空间)\r\n  w/s:关节0\r\n  a/d:关节1\r\n  q/e:关节2\r\n  j/l:关节3\r\n  i/k:关节4\r\n  u/o:关节5')
    print('  夹爪控制：p键')
    print('  控制步长增/减：[/]')
    print('  回到上电位置：0键')
    print('  回到初始位置：9键')
    print('  退出键控程序：Ctrl+C')
    while self.is_alive():
        key = self.key_control(delta_task=0.001,delta_joint=0.01,acc_limit=acc_max,vel_limit=vel_max)
        if key == '9':
            if not args.use_sim:
                self.go_to_named_or_joint_target('PickUnder',sleep_time=2)
            else: self.go_to_named_or_joint_target('PickSim',sleep_time=2)
        elif key == '8':
            self.log_arm_info(target_joint=True,target_pose=True)

        elif key == '0':
            self.go_to_named_or_joint_target('Home')
        elif key == '1':
            # 0.5073,-0.0012,0.0896] rpy: [3.068,-1.529,0.091] xyzw: [-0.722,-0.007,-0.692,0.005]
            position = [0.5073,-0.0012,0.0896]
            rot = [-0.722,-0.007,-0.692,0.005]
            self.set_and_go_to_pose_target(position, rot, '0',sleep_time=2)

        elif key == '2':
            self.gripper_control(1,force=1000,always=True,sleep_time=1.5)

        elif key == 'b':
            self.gripper_control(0,sleep_time=1.5)

        elif key == '3':
            info = self.log_arm_info(current_joint=True,current_pose=True,target_joint=True,target_pose=True,raw_info=True)
            print("pose info: ",info)