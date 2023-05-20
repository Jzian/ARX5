#!/usr/bin/env python3
import rospy
import argparse
from arx5_control.control import RoboticArmAgent

if __name__ == "__main__":
    NODE_NAME = 'arx5_pick_place'
    rospy.init_node(NODE_NAME)
    rospy.loginfo("Initializing {} Node.".format(NODE_NAME))

    parser = argparse.ArgumentParser("ARX PickPlace param set.")
    parser.add_argument('-r','--use_real',action='store_true',help='use sim or real robot')
    parser.add_argument('-np','--not_auto_pick_place',action='store_true',help='whether pick and place')
    parser.add_argument('-na','--not_auto',action='store_true',help='whether auto follow')
    parser.add_argument('-c','--control_mode',default='g_i',help='choose the control mode, defaut to g_i')
    parser.add_argument('-nt','--not_tof',action='store_true',help='whether use tof')
    parser.add_argument('-ng','--not_gripper',action='store_true',help='whether use gripper')
    parser.add_argument('-cp','--control_param',type=int,default=-1,help='whether set control param')
    args, unknown = parser.parse_known_args()

    use_sim = not(args.use_real)  # 是否仿真
    auto_follow = not(args.not_auto)  # 是否自动
    auto_pick_place = not(args.not_auto_pick_place)  # 是否自动夹取
    control_mode = args.control_mode  # 控制模式
    use_tof = not(args.not_tof)  # 是否使用tof
    use_gripper = not(args.not_gripper)  # 是否使用夹爪

    # 初始化机器人体
    arm = RoboticArmAgent(use_sim,control_mode,init_pose=None,node_name=NODE_NAME)
    if args.use_real:
        arm.task_pick_place_param_init(1)  # 真机使用放置模式1
    else: arm.task_pick_place_param_init(0)# 仿真使用放置模式0
    # 传入args.control_param参数，则进行参数配置阻塞以及自动夹取控制
    if args.control_param >= 0: arm.set_control_param(args.control_param)  # 自调参数（包括pick和place）
    if args.control_param == 0: auto_pick_place = False  # 不自动夹取（也即相当于仅调节pick阶段）
    # 自动模式配置
    if auto_pick_place: arm.PickPlace(pick_keyboard=False,place_mode=0,start_base=0)
    elif auto_follow: arm.AutoFollow()
    rospy.loginfo("{} Node Ready.".format(NODE_NAME))
    rospy.spin()