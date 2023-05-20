import rospy,tf_conversions
from geometry_msgs.msg import TransformStamped

target_pub = rospy.Publisher("/target_TF",TransformStamped,queue_size=1)

# 定义发布偏差数据的函数
def target_publish(xyz_bias,rotation_angle):
    target = TransformStamped()
    target.header.stamp = rospy.Time.now()
    target.transform.translation.x = xyz_bias[1] # 图像y（前后方向）实际对应机械臂的x轴，dx
    target.transform.translation.y = xyz_bias[0] # 图像x（左右方向）实际对应机械臂的y轴，dy
    target.transform.translation.z = xyz_bias[2] # 图像z（垂直方向）实际对应机械臂的z轴，dz
    # 角度按45度为界进行正逆转动的转换，保证优弧
    if rotation_angle > 45:
        rotation_angle = rotation_angle - 90
    tf_q = tf_conversions.transformations.quaternion_from_euler(0,0,rotation_angle*math.pi/180)  # pick&place状态下，您以为的末端关节的roll，其实是机械臂的yaw
    target.transform.rotation.x = tf_q[0]
    target.transform.rotation.y = tf_q[1]
    target.transform.rotation.z = tf_q[2]
    target.transform.rotation.w = tf_q[3]
    target_pub.publish(target)

# 发布偏差数据
target_publish(bias_xyz,rotation_angle)

def feedback_callback(self,target_message:TransformStamped):
    pass

rospy.Subscriber("/target_TF",TransformStamped,feedback_callback,queue_size=1)
