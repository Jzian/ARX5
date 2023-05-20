#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from arx5_msgs.msg import JointCommand
from arx5_msgs.msg import JointFeedback
import matplotlib.pyplot as plt
import os
import sys
import signal

def quit(signum, frame):
    sys.exit()

class QUEUE:
    def __init__(self, length):
        self.queue = []
        for i in range(length):
            self.queue.append(0)
    
    def push(self, data):
        self.queue.append(data)
        self.queue.pop(0)

class MONITOR:
    def __init__(self):
        rospy.init_node('arx5_monitor', anonymous=True)
        rospy.Subscriber("arx5/joint_feedback", JointFeedback, self.cmd_callback)

        self.x = []
        self.y = []

        for i in range(6):
            self.y.append(QUEUE(100))
        for i in range(100):
            self.x.append(i)

        signal.signal(signal.SIGINT, quit)                                
        signal.signal(signal.SIGTERM, quit)
        while(1):
            plt.clf()
            for i in range(6):
                plt.plot(self.x , self.y[i].queue, label='motor'+str(i+1)+"'s potision")
                plt.legend()
            plt.pause(0.00001)

    def cmd_callback(self, data):
        p = data.position
        os.system('clear')
        for i in range(len(p)):
            print('index: '+str(i)+', position: '+str(p[i]))
            self.y[i].push(p[i])
            

mnt = MONITOR()