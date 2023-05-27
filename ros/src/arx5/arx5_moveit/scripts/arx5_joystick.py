#!/usr/bin/env python3
import rospy
from arx5_control.control import RoboticArmAgent
import argparse
from sensor_msgs.msg import Joy


class joyControl:
    def __init__(self):
        self.joytopic='joy'
        self.NODE_NAME = 'arx5_moveit_joystick'
        rospy.init_node(self.NODE_NAME)
        self.acc_max,self.vel_max = 1,1
        self.arm = RoboticArmAgent(False,init_pose=None,node_name=self.NODE_NAME)
    def sub_joy(self):
        rospy.Subscriber(self.joytopic,Joy,self.JoyCallback)
        rospy.spin()

    def JoyCallback(self,msg):
        if (msg.buttons[0]): # A  key=9
            self.arm.gripper_control(0,sleep_time=1.5)
        elif (msg.buttons[1]): # B key=8
            self.arm.log_arm_info(target_joint=True,target_pose=True)
        elif ( msg.buttons[10]): # back key=0
            self.arm.go_to_named_or_joint_target('Home')
        elif (msg.buttons[3]): #X key=1
            1
                # 0.5073,-0.0012,0.0896] rpy: [3.068,-1.529,0.091] xyzw: [-0.722,-0.007,-0.692,0.005]
            # position = [0.5073,-0.0012,0.0896]
            # rot = [-0.722,-0.007,-0.692,0.005]
            # self.arm.set_and_go_to_pose_target(position, rot,'0',sleep_time=2)
        elif (msg.buttons[4]): 
            self.arm.gripper_control(1,force=1000,always=True,sleep_time=1.5)
        # else:
        #     key = self.arm.key_get(0)
        #     self.arm.key_control(delta_task=0.001,delta_joint=0.0174,key=key,acc_limit=self.acc_max,vel_limit=self.vel_max)

        
if __name__ == "__main__":
    joyNode=joyControl()
    joyNode.sub_joy()
    

    
    

