#! /usr/bin/env python

import os
import sys
import time
import math
import rospy
from arx5_control.control import RoboticArmAgent
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from arx5_control.srv import arx5_grasp, arx5_graspRequest, arx5_graspResponse
from arx5_control.srv import det, detRequest,detResponse
from arx5_control.srv import MarkerDetResult,MarkerDetResultRequest, MarkerDetResultResponse

from scipy.spatial.transform import Rotation as R

class arm_control:
    def __init__(self, arm:RoboticArmAgent):
        self.arm = arm
        self.end2cam_T = np.array([[ 0, -1, 0, 0.07758467],
                                    [ 1, 0, 0, -0.0484745],
                                    [ 0, 0, 1, 0.08204764],
                                    [ 0, 0.,0.,1.]])

        self.cam2gp_T = np.array([[ 0, -1, 0, 0.07758467],
                                    [ 1, 0, 0, -0.0484745],
                                    [ 0, 0, 1, 0.08204764],
                                    [ 0, 0.,0.,1.]])
        self.initial_pose = [[0.3040, 0.0060, 0.3198],[3.061, -0.272, 0.096]]
        self.poses = [
                    #   [[0.3206, 0.0174, 0.1759],[-3.088, -0.657, 0.181]],
                    #   [[0.3206, 0.0174, 0.1759],[-3.088, -0.657, 0.181]],
                      [[0.3135 ,0.1471 ,0.1647],[-3.069 ,-0.389 ,0.512]],
                      [[0.3938 ,0.0036 ,0.1778],[ 3.087 ,-0.222 ,-0.016]],
                      [[0.3284 ,-0.2696 ,0.1863],[ 3.063 ,-0.023 ,-0.251]],
                        [[0.3072 ,-0.0015 ,0.1885 ],[3.083 ,-0.095 ,-0.028]],
                       [[0.3284 ,-0.2696 ,0.1863],[ 3.063 ,-0.023 ,-0.251]],
                        [[0.4107 ,0.0195 ,0.1916 ],[3.099 ,-0.185 ,-0.073]],
                        [[0.4365 ,0.1776 ,0.1595 ],[3.016 ,-0.354 ,0.564]],
                        ########## shopping
                        #  x:0.4687 y:0.0446 z:0.1858 roll:0.000 pitch:-1.571 yaw:-0.000
                        # x:0.5710 y:-0.0525 z:0.1545 roll:0.000 pitch:-1.571 yaw:-0.000
                        [[0.4687 ,0.0446 ,0.1858 ],[0.000 ,-1.571 ,-3.14]],
                        [[0.5710 ,-0.0525 ,0.1545 ],[0.000 ,-1.571 ,-3.14]],
                     ]


    def trans(self, det_res):
        r1 = det_res.rotat_0
        r2 = det_res.rotat_1
        r3 = det_res.rotat_2
        r4 = det_res.rotat_3
        r5 = det_res.rotat_4
        r6 = det_res.rotat_5
        r7 = det_res.rotat_6
        r8 = det_res.rotat_7
        r9 = det_res.rotat_8
        t1 = det_res.trans_x
        t2 = det_res.trans_y
        t3 = det_res.trans_z

        cam2obj_T = [[r1, r2, r3, t1],
                   [r4, r5, r6, t2],
                   [r7, r8, r9, t3],
                   [0, 0, 0, 1]]
        self.arm.get_current_state()
        # print(self.arm.current_xyzw)
        # print(self.arm.current_rpy)
        base2end_t = np.array([self.arm.current_xyz])
        base2end_R = R.from_quat(self.arm.current_xyzw).as_matrix()
        a=np.array([[0,0,0,1]])
        base2end_T=np.concatenate((np.concatenate((base2end_R,base2end_t.T),axis=1),a),axis=0)
        print(base2end_T)

        target_pose = np.dot(base2end_T, np.dot(self.end2cam_T, cam2obj_T))
        print("target_pose: " ,target_pose)  
        print("current quat: ",self.arm.current_xyzw)
        target_quat = R.from_matrix(target_pose[0:3,0:3]).as_quat()
        target_pos = target_pose[0:3,3:4].T
        
        return target_pose

    def grasp_demo(self, mode):
        rospy.loginfo("Waiting for {} .".format("Marker_det"))
        rospy.wait_for_service('Marker_det')
        rospy.loginfo("Marker_det is already .")

        Marker = rospy.ServiceProxy('Marker_det', det)

        yolo_req = detRequest()
        det_res = detResponse()
        yolo_req.grasp_enable = 0
        yolo_req.label = mode

        det_res = Marker(yolo_req)
        target_pose = self.trans(det_res)

        # target_quat = R.from_matrix(target_pose[0:3,0:3]).as_quat().tolist()
        target_quat = [-0.707, -0.000001, -0.707, 0.0005]
        target_pos = target_pose[0:3,3:4].T.tolist()
        # print(target_quat)
        print(target_pos[0])
        # target_quat = 
        self.arm.get_current_state()
        # print(self.arm.current_xyzw)
        self.arm.set_and_go_to_pose_target(target_pos[0], self.arm.current_xyzw, sleep_time=1)

        response = arx5_graspResponse()
        response.res = True
        return response

    def move_to_tgt_new(self, mode):
        position = self.poses[mode][0]
        rotation = self.poses[mode][1]
        while(1):
            try:
                self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=1)
                break
            except:
                pass
        # rospy.sleep(1)

    def go_back(self):
        position = self.initial_pose[0]
        rotation = self.initial_pose[1]
        self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=1)
        # rospy.sleep(1)

    def go(self, req):
        
        if req.mode == 1:
            response = self.grasp_demo(req.mode)
            return response

        if req.mode == 2:
            position = [0.1051 ,0.0106,0.3604]
            rotation = [3.075,-1.005,0.214]

            self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=1)
            self.move_to_tgt_new(8)
            self.arm.gripper_control(0,sleep_time=0.5) # 1：夹爪闭合，0：夹爪张开
            self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=1)

            response = arx5_graspResponse()
            response.res = True
            return response
 
        if req.mode == 3:
            position = [0.1689 ,0.0025,0.1703 ]
            rotation = [0.000,-1.571,-3.14]    
            arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)

            response = arx5_graspResponse()
            response.res = True
            return response           


if __name__ == "__main__":

    """系统初始化"""
    # ROS节点初始化
    NODE_NAME = 'arm_control'
    rospy.loginfo("Initializing {} Node.".format(NODE_NAME))
    rospy.init_node(NODE_NAME)
    # 是否仿真
    # use_sim = rospy.get_param("~use_sim", default=False)
    # 设置不自动、不夹取、默认控制模式
    auto_follow = False
    pick_place_enable = False
    control_mode = 'g_i'
    use_gripper = False
    rospy.loginfo("{} Node Ready.".format(NODE_NAME))
    # 初始化机器人实例
    arm = RoboticArmAgent(use_sim=False,node_name=NODE_NAME)
    arm_control_instance = arm_control(arm)
    # position = [0.1051 ,0.0106,0.3604 ]
    # rotation = [3.075,-1.005,0.214]  x:0.1689 y:0.0025 z:0.1703 roll:0.000 pitch:-1.571 yaw:-0.000
    position = [0.1689 ,0.0025,0.1703 ]
    rotation = [0.000,-1.571,-3.14]    
    arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)
    arm.gripper_control(0,sleep_time=1.5)
    
    server = rospy.Service("arx_server",arx5_grasp,arm_control_instance.go)

    rospy.spin()

