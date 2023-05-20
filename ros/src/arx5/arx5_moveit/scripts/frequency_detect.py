import rospy
import arx5_msgs.msg

# ROS节点初始化
NODE_NAME = 'frequency_detect'
rospy.init_node(NODE_NAME)
last = 0
max_d = 6*1000000

def cb(message:arx5_msgs.msg.JointCommand):
    global last
    if last == 0:
        last = message.header.stamp.nsecs
    else:
        now = message.header.stamp.nsecs
        delta = now - last
        last = now
        if delta > max_d: rospy.loginfo(f'{delta}')

rospy.Subscriber('arx5/joint_command', arx5_msgs.msg.JointCommand, cb ,queue_size=10)
rospy.spin()
