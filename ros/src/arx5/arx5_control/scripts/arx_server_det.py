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
from scipy.spatial.transform import Rotation as R

class arm_control:
    def __init__(self, arm:RoboticArmAgent):
        self.arm = arm
        # self.end2cam_T = np.array([[ 0, -1, 0, 0.07758467],
        #                             [ 1, 0, 0, -0.0484745],
        #                             [ 0, 0, 1, 0.08204764],
        #                             [ 0, 0.,0.,1.]])
        # self.cam2hand_T = np.array([[ 0, 0, 1, -0.092],
        #                             [ 1, 0, 0, 0.053],
        #                             [ 0, 1, 0, -0.085],
        #                             [ 0, 0.,0.,1.]])
        # self.cam2hand_T = np.array([[ 0, 1, 0, -0.08],
        #                             [ -1, 0, 0, 0.05],
        #                             [ 0, 0, 1, -0.095],
        #                             [ 0, 0.,0.,1.]])
        # self.cam2hand_T = np.array([[ 0, -1, 0, 0.075],
        #                             [ 1, 0, 0, 0.015],
        #                             [ 0, 0, 1, -0.095],
        #                             [ 0, 0.,0.,1.]])
        self.cam2hand_T = np.array([[ 0, -1,  0,  0.073],
                                    [ 1,  0,  0,  0.009],
                                    [ 0,  0,  1, -0.105],
                                    [ 0,  0,  0,  1     ]])      

        # self.cam2hand_T = np.array([[ 0, 1, 0, -0.072],  
        #                             [ -1, 0, 0, -0.012],
        #                             [ 0, 0, 1, -0.095],
        #                             [ 0, 0.,0.,1.]])
        # self.cam2gp_T = np.array([[ 0, -1, 0, 0.07758467],
        #                             [ 1, 0, 0, -0.0484745],
        #                             [ 0, 0, 1, 0.08204764],
        #                             [ 0, 0.,0.,1.]])
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

    # x:0.3284 y:-0.2696 z:0.1863 roll:3.063 pitch:-0.023 yaw:-0.251 nadao

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

        obj2cam_T = np.array([[r1, r2, r3, t1],
                   [r4, r5, r6, t2],
                   [r7, r8, r9, t3+0.05],
                   [0, 0, 0, 1]])
        print("yolo_pose", obj2cam_T)
        # self.arm.get_current_state()
        # print("current_xyzw", self.arm.current_xyzw)
        # print("current_rpy", self.arm.current_rpy)
        # print("current_xyz", self.arm.current_xyz)
        hand2base_T = np.array([self.arm.current_xyz])
        base2hand_R = R.from_quat(self.arm.current_xyzw).as_matrix()
        a=np.array([[0,0,0,1]])
        hand2base_T=np.concatenate((np.concatenate((base2hand_R,hand2base_T.T),axis=1),a),axis=0)
        # print(hand2base_T)
        # tar = np.dot(self.cam2hand_T, obj2cam_T)
        # print("hand2tar: ", tar)
        target_pose = np.dot(hand2base_T, np.dot(self.cam2hand_T, obj2cam_T))
        print("target_pose: " ,target_pose)  
        # print("current quat: ",self.arm.current_xyzw)
        target_quat = R.from_matrix(target_pose[0:3,0:3]).as_quat()
        target_pos = target_pose[0:3,3:4].T
        
        # print(target_quat)
        # print(target_pos)
        return target_pose

    def grasp_sauce_bottle(self, mode):
        self.arm.gripper_control(0,sleep_time=0.5)
        rospy.loginfo("Waiting for {} .".format("yolo_det"))
        rospy.wait_for_service('yolo_det')
        rospy.loginfo(" {} is already .".format("yolo_det"))
        yolo = rospy.ServiceProxy('yolo_det', det)
        yolo_req = detRequest()
        det_res = detResponse()
        yolo_req.grasp_enable = 0
        yolo_req.label = 16
        det_res = yolo(yolo_req)
        # if det_res.rotat_0 == 1:
        while np.abs(det_res.trans_z) < 0.05 or np.abs(det_res.trans_z) > 0.5:
            det_res = yolo(yolo_req)
        target_pose = self.trans(det_res)

        target_quat = R.from_matrix(target_pose[0:3,0:3]).as_quat().tolist()
        # target_quat_arm = [-0.707, -0.000001, -0.707, 0.0005]
        target_pos = target_pose[0:3,3:4].T.tolist()
        # print(target_quat)
        print(target_pos[0])
        # target_quat = 
        self.arm.get_current_state()
        # print(self.arm.current_xyzw)
        print("target_rpy : ", self.arm.current_rpy)
        print("current xyzw: ", self.arm.current_xyzw)
        print("target_quat xyzw: ", target_quat)
        print("target_pos : ", target_pos)
        self.arm.set_and_go_to_pose_target([target_pos[0][0]-0.07,target_pos[0][1],target_pos[0][2]], target_quat, sleep_time=0.5)
        self.arm.set_and_go_to_pose_target(target_pos[0], target_quat, sleep_time=0.5)
        self.arm.gripper_control(1,sleep_time=0.5, force=800) # 1：夹爪闭合，0：夹爪张开
        target_pos_mid = [0.4013,target_pos[0][1],target_pos[0][2]+0.1]
        self.arm.set_and_go_to_pose_target(target_pos_mid, target_quat, sleep_time=0.5)
        position = [0.3613 ,0.0023,0.1702 ]
        rotation = [1.571,-1.571,1.571]  
        self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=0.5)
        # self.trans(det_res)
        
        # print(det_res.trans_x)
        # print(det_res.trans_y)
        # print(det_res.trans_z)

        # print(det_res.center_x)
        # print(det_res.center_y)
        # print(det_res.center_z)
        response = arx5_graspResponse()
        response.res = True
        return response

    def grasp_seasoning_bottle(self, mode):
        self.arm.gripper_control(0,sleep_time=0.5)
        rospy.loginfo("Waiting for {} .".format("yolo_det"))
        rospy.wait_for_service('yolo_det')
        rospy.loginfo(" {} is already .".format("yolo_det"))
        yolo = rospy.ServiceProxy('yolo_det', det)
        yolo_req = detRequest()
        det_res = detResponse()
        yolo_req.grasp_enable = 0
        yolo_req.label = 17
        det_res = yolo(yolo_req)
        while np.abs(det_res.trans_z) < 0.05 or np.abs(det_res.trans_z) > 0.5:
            det_res = yolo(yolo_req)
        target_pose = self.trans(det_res)

        target_quat = R.from_matrix(target_pose[0:3,0:3]).as_quat().tolist()
        # target_quat_arm = [-0.707, -0.000001, -0.707, 0.0005]
        target_pos = target_pose[0:3,3:4].T.tolist()
        # print(target_quat)
        print(target_pos[0])
        # target_quat = 
        self.arm.get_current_state()
        # print(self.arm.current_xyzw)
        print("target_rpy : ", self.arm.current_rpy)
        print("current xyzw: ", self.arm.current_xyzw)
        print("target_quat xyzw: ", target_quat)
        print("target_pos : ", target_pos)
        self.arm.set_and_go_to_pose_target([target_pos[0][0]-0.07,target_pos[0][1],target_pos[0][2]], target_quat, sleep_time=0.5)
        self.arm.set_and_go_to_pose_target(target_pos[0], target_quat, sleep_time=0.5)
        self.arm.gripper_control(1,sleep_time=0.5, force=800) # 1：夹爪闭合，0：夹爪张开
        target_pos_mid = [0.4013,target_pos[0][1],target_pos[0][2]+0.10]
        self.arm.set_and_go_to_pose_target(target_pos_mid, target_quat, sleep_time=0.5)

        position = [0.3613 ,0.0023,0.1702 ]
        rotation = [1.571,-1.571,1.571]  
        self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=0.5)
        # self.trans(det_res)
        
        # print(det_res.trans_x)
        # print(det_res.trans_y)
        # print(det_res.trans_z)

        # print(det_res.center_x)
        # print(det_res.center_y)
        # print(det_res.center_z)
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
            response = self.grasp_sauce_bottle(req.mode)
            return response
            # position = [0.1051 ,0.0106,0.3604]
            # rotation = [3.075,-1.005,0.214]

            # self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)
            # self.move_to_tgt_new(7)
            # self.arm.gripper_control(1,sleep_time=0.5) # 1：夹爪闭合，0：夹爪张开
            # self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=1)

            # response = arx5_graspResponse()
            # response.res = True
            # return response

        if req.mode == 2:
            response = self.grasp_seasoning_bottle(req.mode)
            return response
            
            # position = [0.1051 ,0.0106,0.3604]
            # rotation = [3.075,-1.005,0.214]

            # self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=1)
            # self.move_to_tgt_new(8)
            # self.arm.gripper_control(0,sleep_time=0.5) # 1：夹爪闭合，0：夹爪张开
            # self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=1)

            # response = arx5_graspResponse()
            # response.res = True
            # return response
 
        if req.mode == 3:
           #[0.7197,-0.0181,0.1154]  xyzw: [0.694,0.004,0.719,0.024]
            position = [0.7197,-0.0181,0.2054]
            rotation = [1.571,-1.571,1.571]
            arm.set_and_go_to_pose_target(position,rotation,sleep_time=1.5)
            self.arm.gripper_control(0,sleep_time=1.0) # 1：夹爪闭合，0：夹爪张开
            position_back = [0.3613 ,0.0023,0.1702 ]
            rotation_back = [1.571,-1.571,1.571]        
            arm.set_and_go_to_pose_target(position_back,rotation_back,sleep_time=1.5)       
            response = arx5_graspResponse()
            response.res = True
            return response     

        if req.mode == 4:
           #[0.7197,-0.0181,0.1154]  xyzw: [0.694,0.004,0.719,0.024]
            position = [0.7197,-0.1581,0.1554]
            rotation = [1.571,-1.571,1.571]
            arm.set_and_go_to_pose_target(position,rotation,sleep_time=1.5)
            self.arm.gripper_control(0,sleep_time=1.0) # 1：夹爪闭合，0：夹爪张开
            position_back = [0.3613 ,0.0023,0.1702 ]
            rotation_back = [1.571,-1.571,1.571]        
            arm.set_and_go_to_pose_target(position_back,rotation_back,sleep_time=1.5)       
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
    # rotation = [3.075,-1.005,0.214] 0.3613,0.0023,0.1702] rpy: [1.694,-1.497,1.426
    # position = [0.1689 ,0.0025,0.1703 ]
    # rotation = [0.000,-1.571,-3.14]
    position = [0.3613 ,0.0023,0.1702 ]
    rotation = [1.571,-1.571,1.571]     
    arm.get_current_state()
    print("current_xyzw", arm.current_xyzw)
    print("current_rpy", arm.current_rpy)
    print("current_xyz", arm.current_xyz)
    arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)
    arm.gripper_control(0,sleep_time=1.5)
    
    server = rospy.Service("arx_server",arx5_grasp,arm_control_instance.go)
    rospy.loginfo(" {} is already .".format("yolo_det"))

    rospy.spin()



# 0. 拿西红柿 夹爪抓  x:0.1172 ,0.0020 ,0.1563 roll:0.000 ,-1.571 ,-0.000 rot_x:-0.707 rot_,-0.000 rot_,-0.707 rot_w:0.000 
        # 1. 把西红柿放砧板上 松 x:0.1172 ,0.0020 ,0.1563 roll:0.000 ,-1.571 ,-0.000 rot_x:-0.707 rot_,-0.000 rot_,-0.707 rot_w:0.000 (m/rad
        # 2. 拿刀 抓 x:0.1172 ,0.0020 ,0.1563 roll:0.000 ,-1.571 ,-0.000 rot_x:-0.707 rot_,-0.000 rot_,-0.707 rot_w:0.000
        # 3. 切西红柿 x:0.1172 ,0.0020 ,0.1563 roll:0.000 ,-1.571 ,-0.000 rot_x:-0.707 rot_,-0.000 rot_,-0.707 rot_w:0.000 (m/rad
        # 4. 放刀 放
        # 5. 抓切好的西红柿 抓 x:0.1172 ,0.0020 ,0.1563 roll:0.000 ,-1.571 ,-0.000 rot_x:-0.707 rot_,-0.000 rot_,-0.707 rot_w:0.000 (m/rad
        # 6. 放盘子里 放
        
        # if req.mode == 9:
            # # 过去夹再回来
            # self.go_back()
            # self.move_to_tgt_new(0)
            # self.arm.gripper_control(1,sleep_time=1.5) # 1：夹爪闭合，0：夹爪张开
  
            
        
            # # 过去放再回来
            # self.go_back()
            # self.move_to_tgt_new(1)
            # self.arm.gripper_control(0,sleep_time=1.5) # 1：夹爪闭合，0：夹爪张开


        
            # self.go_back()

            # position = [0.3127 ,-0.2592,0.2444]
            # rotation = [3.010,-0.132,-0.251]
            # self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)

            # self.move_to_tgt_new(2)
            # self.arm.gripper_control(1,sleep_time=1.5,force=500) # 1：夹爪闭合，0：夹爪张开

            # position = [0.3127 ,-0.2592,0.2444]
            # rotation = [3.010,-0.132,-0.251]
            # self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)
            
    
            # # qie
            # self.go_back()
            # self.move_to_tgt_new(3)
            # # self.arm.gripper_control(0,sleep_time=1.5) # 1：夹爪闭合，0：夹爪张开
            # self.go_back()
            
            # position = [0.3127 ,-0.2592,0.2444]
            # rotation = [3.010,-0.132,-0.251]
            # self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)
            # self.move_to_tgt_new(4)

            # self.arm.gripper_control(0,sleep_time=1.5) # 1：夹爪闭合，0：夹爪张开

            # position = [0.3127 ,-0.2592,0.2444]
            # rotation = [3.010,-0.132,-0.251]
            # self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)

            # self.go_back()
            
       
            # self.go_back()
            # self.move_to_tgt_new(5)
            # self.arm.gripper_control(1,sleep_time=1.5) # 1：夹爪闭合，0：夹爪张开
            # self.go_back()
            

        
            # self.go_back()
            # self.move_to_tgt_new(6)
            # self.arm.gripper_control(0,sleep_time=1.5) # 1：夹爪闭合，0：夹爪张开
            # self.go_back()
            # response = arx5_graspResponse()
            # response.res = True
            # return response