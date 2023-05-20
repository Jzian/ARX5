from arx5_control.control import RoboticArmAgent
from arx5_vision.vision import StdVideo
import rospy
from sensor_msgs.msg import Image,CameraInfo
from typing import Optional,Union,List,Tuple,Dict,Any,Callable,Iterable
from numpy import ndarray
from cv_bridge import CvBridge
import cv2

"""
    在这个例程中，您将看到如何进行图像处理获取物块相关的参数。 
        在主程序中，首先完成了视觉参数、ROS节点等的配置，然后获取了相机的参数信息。
        然后根据仿真或实际进行获取图像方式的选择。若是仿真，则将控制机械臂到达待识别的位置，然后开启识别。
        然后在image_process函数中完成对图像的处理。
"""

# 图像处理函数
def image_process(frame):
    frame  = CvBridge().imgmsg_to_cv2(frame, "bgr8")  # Image格式转换为cv2格式
    framecp = frame.copy()

    HSV_L,HSV_H = [64,93,59],[179,255,255]
    _,binary_img = StdVideo.color_filter(framecp,HSV_L,HSV_H)

    contours = StdVideo.Contour.find(binary_img)
    center_xy,bias_xyz,rotation_angle,box_int,w_h,cnt = StdVideo.Contour.NFC_F(contours,(4000,60000),3.0,(ImageSize[0]/2,529,20000))
    if box_int is not None:
        StdVideo.Contour.draw(frame,[box_int])
    StdVideo.Show('output',[frame,binary_img],window_size=(640,320),wait_time=1)


if __name__ == '__main__':

    # 初始化节点
    NODE_NAME = 'cube_vision'
    rospy.init_node(NODE_NAME)
    # 初始化机器人实例
    arm = RoboticArmAgent(use_sim=True,node_name=NODE_NAME,init_pose=2)

    """ 获取相机参数，主要是用到了宽和高来作为参考"""
    camrera_info:CameraInfo = rospy.wait_for_message("/camera/color/camera_info",CameraInfo,timeout=2)
    ImageSize = [camrera_info.width,camrera_info.height]

    # 订阅仿真中的相机图像
    rospy.Subscriber("camera/color/image_raw",Image,image_process,queue_size=1)
    # 阻塞直到节点退出
    rospy.spin()
