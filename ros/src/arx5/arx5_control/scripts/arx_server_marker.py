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
from arx5_control.srv import MarkerDetResult,MarkerDetResultRequest, MarkerDetResultResponse

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
        # self.cam2hand_T = np.array([[ 0, -1,  0,  0.062],   #send
        #                             [ 1,  0,  0, -0.01],
        #                             [ 0,  0,  1, -0.104],
        #                             [ 0,  0,  0,  1     ]])
        # self.cam2hand_T = np.array([[ 0, 1, 0, -0.072],  
        #                             [ -1, 0, 0, -0.012],
        #                             [ 0, 0, 1, -0.095],
        #                             [ 0, 0.,0.,1.]])
        # self.cam2gp_T = np.array([[ 0, -1, 0, 0.07758467],
        #                             [ 1, 0, 0, -0.0484745],
        #                             [ 0, 0, 1, 0.08204764],
        #                             [ 0, 0.,0.,1.]])
        self.initial_pose = [[0.3040, 0.0060, 0.3198],[3.061, -0.272, 0.096]]

        self.poses_preset = {
                    'init_pose': {'pos': [0.3213 ,0.0023,0.1702], 'rot': [1.571,-1.571,1.571]}, # init pose
                    'put_pose': {'pos': [0.6527701,0.02,0.20748018], 'rot': [1.571,-1.571,1.571]}, # init pose
        }
        self.grasp_height = 0.11
        self.put_height = 0.083
        self.obj_center_height = 0.05

        self.grasp_list = [0,1,2,3,4]
        self.put_list = [5,6,7,8,9]
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
                   [r7, r8, r9, t3+0.03],
                   [0, 0, 0, 1]])
        print("yolo_pose", obj2cam_T)
        self.arm.get_current_state()
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

    def get_marker_pose(self,mode):
        rospy.loginfo("Waiting for {} .".format("Marker_det"))
        rospy.wait_for_service('Marker_det')
        rospy.loginfo("Marker_det is already. ")

        Marker = rospy.ServiceProxy('Marker_det', MarkerDetResult)
        marker_req = MarkerDetResultRequest()
        marker_res = MarkerDetResultResponse()
        marker_req.size = 0.03
        marker_req.id = mode
        unview_idx = 0
        while True:
            marker_res = Marker(marker_req)
            if marker_res.idexist == 0:
                unview_idx += 1
                print("unview: ", unview_idx)
                position_get = self.poses_preset['init_pose']['pos'][:]
                rotation_get = self.poses_preset['init_pose']['rot'][:]
                if unview_idx < 3:
                    position_get[1] = position_get[1] + 0.05*unview_idx
                elif unview_idx > 2 and unview_idx < 5:
                    position_get[1] = position_get[1] - 0.05*(unview_idx-2)
                else:
                    rospy.logerr("cannot find marker!")
                    return 0, 0 
                flag = arm.set_and_go_to_pose_target(position_get, rotation_get, sleep_time=0.5)
                print("flag",flag)
                rospy.sleep(0.1)
            else:
                break
        
        # if det_res.rotat_0 == 1:
        # while np.abs(det_res.trans_z) < 0.05 or np.abs(det_res.trans_z) > 0.5:
        #     det_res = yolo(yolo_req)
        target_pose = self.trans(marker_res)


        target_quat = R.from_matrix(target_pose[0:3,0:3]).as_quat().tolist()
        # target_quat_arm = [-0.707, -0.000001, -0.707, 0.0005]
        target_pos = target_pose[0:3,3:4].T.tolist()
        # print(target_quat)
        print(target_pos[0])
        # target_quat = 
        self.arm.get_current_state()
        print("current_xyzw", self.arm.current_xyzw)
        # print("target_rpy : ", self.arm.current_rpy)
        # print("current xyzw: ", self.arm.current_xyzw)
        # print("target_quat xyzw: ", target_quat)
        print("target_pos : ", target_pos)
        target_quat = self.arm.current_xyzw
        return target_pos, target_quat

    def grasp(self, mode):
        self.arm.gripper_control(0,sleep_time=0.5)

        # get maker pose
        target_pos, target_quat = self.get_marker_pose(mode)
        if target_pos == 0 and target_quat == 0:
            self.arm.set_and_go_to_pose_target(self.poses_preset['init_pose']['pos'],self.poses_preset['init_pose']['rot'],sleep_time=0.5)
            response = arx5_graspResponse()
            response.res = False
            return response
            
        self.obj_center_height = np.abs(target_pos[0][2] - self.grasp_height) 
        print("obj_center_height: ", self.obj_center_height)

        self.arm.set_and_go_to_pose_target([target_pos[0][0]-0.06,target_pos[0][1],target_pos[0][2]], self.poses_preset['init_pose']['rot'], sleep_time=0.5)

        # target_pos, target_quat = self.get_marker_pose(mode)

        self.arm.set_and_go_to_pose_target(target_pos[0], self.poses_preset['init_pose']['rot'], sleep_time=0.5)

        self.arm.gripper_control(1,sleep_time=0.5, force=800) # 1：夹爪闭合，0：夹爪张开

        target_pos_mid = [0.3413, target_pos[0][1],target_pos[0][2]+0.1]
        self.arm.set_and_go_to_pose_target(target_pos_mid, target_quat, sleep_time=0.5)

        self.arm.set_and_go_to_pose_target(self.poses_preset['init_pose']['pos'],self.poses_preset['init_pose']['rot'],sleep_time=0.5)

        response = arx5_graspResponse()
        response.res = True
        return response

    def put(self, mode):

        print("obj_center_height in put : ", self.obj_center_height)
        position_put = self.poses_preset['put_pose']['pos'][:]
        rotation_put = self.poses_preset['put_pose']['rot'][:]
        position_mid = position_put[:]

        position_mid[0] = position_put[0] - 0.15
        position_mid[1] = position_put[1] + 0.08*(7-mode)
        position_mid[2] = self.put_height + self.obj_center_height+0.03

        position_put[1] = position_put[1] + 0.08*(7-mode)
        position_put[2] = self.put_height + self.obj_center_height+0.005
        
        arm.set_and_go_to_pose_target(position_mid, rotation_put, sleep_time=0.5)

        arm.set_and_go_to_pose_target(position_put, rotation_put, sleep_time=0.5)
        self.arm.gripper_control(0,sleep_time=1.0) # 1：夹爪闭合，0：夹爪张开

        arm.set_and_go_to_pose_target(position_mid, rotation_put, sleep_time=0.5)

        arm.set_and_go_to_pose_target(self.poses_preset['init_pose']['pos'],self.poses_preset['init_pose']['rot'],sleep_time=0.5)       
        response = arx5_graspResponse()
        response.res = True
        return response   



    def go_back(self):
        position = self.initial_pose[0]
        rotation = self.initial_pose[1]
        self.arm.set_and_go_to_pose_target(position,rotation,sleep_time=1)
        # rospy.sleep(1)

    def go(self, req):
        if req.mode in self.grasp_list:
            response = self.grasp(req.mode)
            return response

        if req.mode in self.put_list:
            response = self.put(req.mode)
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
    position = [0.3213 ,0.0023,0.1702 ]
    rotation = [1.571,-1.571,1.571]     
    arm.get_current_state()
    print("current_xyzw", arm.current_xyzw)
    print("current_rpy", arm.current_rpy)
    print("current_xyz", arm.current_xyz)
    arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)
    arm.gripper_control(0,sleep_time=1.5)
    
    server = rospy.Service("arx_server",arx5_grasp,arm_control_instance.go)
    rospy.loginfo(" {} is already .".format("Marker_based_grasp"))

    rospy.spin()

