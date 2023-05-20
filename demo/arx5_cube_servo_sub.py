import rospy,tf_conversions
from geometry_msgs.msg import TransformStamped
import math
from arx5_control.control import RoboticArmAgent
import numpy as np

# ROS节点初始化
NODE_NAME = 'arm_control'
rospy.init_node(NODE_NAME)
# 初始化机器人实例
arm = RoboticArmAgent(use_sim=True,node_name=NODE_NAME,init_pose=2)
# 定义回调函数
def feedback_callback(target_message:TransformStamped):
    """ 相对偏差转绝对偏差 """
    arm.get_current_state()  # 得到当前状态
    feedback_target_position = [target_message.transform.translation.x,target_message.transform.translation.y,target_message.transform.translation.z]
    feedback_target_euler = list(tf_conversions.transformations.euler_from_quaternion([target_message.transform.rotation.x,target_message.transform.rotation.y,
                                                            target_message.transform.rotation.z,target_message.transform.rotation.w]))
    # 跟踪色块时(且是pick阶段)根据 此时的yaw偏角（应注意，世界坐标系下，此时末端关节的转动不再是roll而是yaw） 对trans进行额外处理，转换xy相对世界坐标系为相对夹爪朝向本身
    # 求出yaw的偏差
    current_yaw_bias = -(arm._pick_rpy[2] - arm.current_rpy[2])  # 以顺时针偏转时该bias为正值代入后续计算
    # 根据当前的roll偏差解算出实际应该移动的前后和左右（z轴和y轴）距离（近似认为当前roll和反馈的数据是同步的）
    tf_trans_x =  feedback_target_position[0]*math.cos(current_yaw_bias) - feedback_target_position[1]*math.sin(current_yaw_bias)  # 左右移动
    feedback_target_position[1] = feedback_target_position[0]*math.sin(current_yaw_bias) + feedback_target_position[1]*math.cos(current_yaw_bias)  # 前后移动
    feedback_target_position[0] = tf_trans_x  # 临时保存的用完

    """ 像素坐标转世界坐标 """

# 创建话题订阅者
rospy.Subscriber("/target_TF",TransformStamped,feedback_callback,queue_size=1)
rospy.spin()
