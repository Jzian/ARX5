#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import arx5_msgs.msg

test_mode = 'position'

rospy.init_node('tst_single_antuator', anonymous=True)
cmd_publisher = rospy.Publisher('arx5/joint_command', arx5_msgs.msg.JointCommand, queue_size=10)

cmd = arx5_msgs.msg.JointCommand()
cmd.header.stamp = rospy.Time.now()
cmd.header.frame_id = 'arx5'

if(test_mode == 'torque'):
    cmd.mode = 'torque'
    cmd.id = [1]
    cmd.kp = [0]
    cmd.kd = [1]
    cmd.position = [0]
    cmd.velocity = [0]
    cmd.effort = [1]

elif(test_mode == 'position'):
    cmd.mode = 'position'
    cmd.id = [2]
    cmd.kp = [0]
    cmd.kd = [0]
    cmd.position = [5.0]
    cmd.velocity = [0]
    cmd.effort = [0]

elif(test_mode == 'PD'):
    cmd.mode = 'PD'
    cmd.id = [1]
    cmd.kp = [0]
    cmd.kd = [0]
    cmd.position = [360]
    cmd.velocity = [0]
    cmd.effort = [0]

elif(test_mode == 'test'):
    cmd.mode = 'test'
    cmd.id = [1]
    cmd.kp = [0]
    cmd.kd = [0]
    cmd.position = [0]
    cmd.velocity = [0]
    cmd.effort = [0]

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    cmd_publisher.publish(cmd)
    rate.sleep()