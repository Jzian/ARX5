#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray,AprilTagDetection
from geometry_msgs.msg import TransformStamped, Transform ,Point32

from cv_bridge import CvBridge
import cv2

import numpy as np

import threading
import os
import math

NODE_NAME = 'arx5_vision'  # TODO：后续该文件改名成arx5_tag_detect

def loginfo(msg: str):
    rospy.loginfo("[{}] {}".format(NODE_NAME, msg))

def logwarn(msg: str):
    rospy.logwarn("[{}] {}".format(NODE_NAME, msg))

def transformAsMatrix(trans: Transform):
    trans_q = trans.rotation
    trans_R = tf_conversions.transformations.quaternion_matrix(
        [trans_q.x, trans_q.y, trans_q.z, trans_q.w])
    trans_t = tf_conversions.transformations.translation_matrix(
        [trans.translation.x, trans.translation.y, trans.translation.z])
    trans_T = tf_conversions.transformations.concatenate_matrices(
        trans_t, trans_R)
    return trans_T

def makedir(dir_path):
    dir_path=os.path.dirname(dir_path)#获取路径名，删掉文件名
    bool=os.path.exists(dir_path) #存在返回True，不存在返回False
    if bool:
        pass
    else:
        os.makedirs(dir_path)


class TagDetectNode:
    def __init__(self) -> None:
        rospy.init_node(NODE_NAME)
        loginfo("Initializing {} node.".format(NODE_NAME))

        self.use_sim = rospy.get_param("~use_sim", default=False)
        self.image_save_flag = rospy.get_param("~image_save", default=False)
        self.sub_method = 2  # 0表示订阅tag_detection；1表示直接订阅TF。前者适合相机跟踪时用，后者适合手眼标定后夹爪对中时用。2表示都订阅。

        self.Image = None
        self.new_image = True
        self.topic_name = rospy.get_param("~image_topic", default="")
        self.base_frame = rospy.get_param("~base_frame", default="")
        self.target_tag_id = int(rospy.get_param("~target_tag", default=0))
        self.target_dis = float(rospy.get_param("~target_distance", default=0.2))

        if self.image_save_flag:
            # 开启图片订阅
            self.img_sub = rospy.Subscriber(self.topic_name, Image, self.img_callback,queue_size=1)
            loginfo("开启RGB图片接收程序\r\n")
            # color图片保存线程
            img_save_thread = threading.Thread(target = self.save_images,args=(2,0.5))
            img_save_thread.setDaemon(True)
            img_save_thread.start()
            loginfo("开启RGB图片保存线程\r\n")

        self.TargetTF = TransformStamped()
        self.TargetTF.header.frame_id = self.base_frame
        self.TargetTF.child_frame_id = "tag_{}_target".format(self.target_tag_id)
        # 话题订阅模式选择
        if self.sub_method==0 or self.sub_method==2:
            # 订阅tag识别话题
            self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)
            loginfo("target_tag: {}, base_frame: {}\r\n".format(self.target_tag_id, self.base_frame))
        elif self.sub_method==1 or self.sub_method==2:
            # 初始化监听tf
            self.tfBuffer = tf2_ros.Buffer(rospy.Duration(2)) # 缓存变换数据
            self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # 创建发布目标相对距离信息的topic（topic消息队列可控，比TF发布更具有实时性）
        self.target_pub = rospy.Publisher("/target_TF",TransformStamped,queue_size=1) # queue_size=1表明只发布最新数据
        loginfo("publish to /target_TF\r\n")

    # 回调函数中处理尽可能少的数据（保存并转换图像，清零标志位）
    def img_callback(self, img: Image):
        self.Image = img
        self.CVimage = CvBridge().imgmsg_to_cv2(self.Image, "bgr8")
        self.new_image = 0

    # 可以试图通过exif获取图像的参数并进行发布，取决于相机是否有保存相关数据，不一定成功
    def get_and_pub_camera_info(self):
        pass

    # # 根据颜色及边缘找到图像轮廓信息
    # def process_images(self):
    #     # 线程中循环处理
    #     while True:
    #         if self.new_image[0][0]==0:  # 利用一个标志列表控制多线程防止对同一幅图重复处理
    #             self.new_image[0][0] = 1
    #             # # 根据仿真与否配置不同的参数
    #             # if not self.use_sim:
    #             #     # 非仿真模式额外进行一些图片的预处理
    #             #     blur_laplace = cv2.Laplacian(self.CVimage, -1)  # 高斯滤波
    #             #     self.CVimage = cv2.addWeighted(self.CVimage, 1, blur_laplace, -0.5, 0)  # 两幅图叠加
    #             #     canny_param = 'Real'
    #             # else:
    #             #     canny_param = 'Sim'
    #             # # 首先根据颜色确定基本掩模，根据颜色掩模得到去除杂散颜色后的新的color图（基本ROI，颜色阈值范围比较宽泛，目的是在不损失目标颜色物体的前提下，去掉大多数不同颜色的其它物体）
    #             # res, mask = Color_Chooser(self.CVimage,self.HSV_Corlor['YellowL'],self.HSV_Corlor['YellowH'])
    #             # # 将新的颜色图进行灰度化
    #             # res_gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    #             # # 对灰度化图进行canny边缘检测（canny自带了5x5高斯滤波器消除图像中的噪声）
    #             # edges = cv2.Canny(res_gray,self.CannyParam[canny_param][0],self.CannyParam[canny_param][1],self.CannyParam[canny_param][2],self.CannyParam[canny_param][3])
    #             # # 进行轮廓检测（接受的参数为二值图,常用canny边缘图）
    #             # contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 将图像消息保存为OpenCV格式(path的路径必须是../开头，否则会无法保存，不知道是什么bug)
    def save_images(self,num=1,wait=0,path="../Images/",cover=True,diff=False):
        i=0
        while True:  # 线程中一直循环
            if self.new_image == 0:
                self.new_image = 1
                cvimg = CvBridge().imgmsg_to_cv2(self.Image, "bgr8")
                if cover==True:
                    image_name = str(i) + "_Image_static3.jpg"
                    cv2.imwrite(path + image_name, cvimg)  # 重复写入同名文件，保证图像始终为最新，同时避免文件爆炸
                    if diff is True and i>0:
                        # 将当前图像和上一贞图像作差（上一贞图像通过imread获取）
                        difimg = cv2.absdiff(cvimg, cv2.imread(path + str(i-1) + "_Image.jpg"))
                        cv2.imwrite(path + "Diff_" + image_name, difimg)  # 保存贞差图
                else:  # 非覆盖情况暂不支持直接保存diff图
                    timestr = "_%.5f" %  self.Image.header.stamp.to_sec() # %.6f表示小数点后带有6位，可根据精确度需要修改
                    image_name = str(i) + timestr+ ".jpg" # 图像命名：时间戳.jpg
                    cv2.imwrite(path + image_name, cvimg)  # 保存
                # loginfo("图片{}保存完成\r\n".format(i))
                if wait > 0:
                    rospy.sleep(wait)
                i+=1
                if i==num:
                    i=0

    # 接收到标签识别到的话题并进行处理后将相对偏差通过话题进行发布
    def tag_callback(self, tags: AprilTagDetectionArray):
        header = tags.header
        tag:AprilTagDetection  # 指明tag类型
        for tag in tags.detections:
            if tag.id[0] == self.target_tag_id:
                self.TargetTF.header.stamp = header.stamp
                # 实机
                if not self.use_sim:
                    trans:TransformStamped = TransformStamped()
                    try:
                        # 从tf缓存中找到所需时间的变换信息（事实上如果只是获取tag和camera的tf完全可以通过tag类中的相关成员;这里tf的根本作用还是直接获得目标-夹爪的TF变换关系）
                        trans = self.tfBuffer.lookup_transform(
                            "tag_{}".format(tag.id[0]),
                            self.base_frame,
                            rospy.Duration(0) , rospy.Duration(2))
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        loginfo("TF变换查找警告:")
                        logwarn(e)
                        return

                    # TF变换处理
                    trans_T = transformAsMatrix(trans.transform)
                    target_R = tf_conversions.transformations.quaternion_matrix([0.500, 0.500, -0.500, 0.500])
                    target_t = tf_conversions.transformations.translation_matrix([0, 0, self.target_dis])
                    target_T = tf_conversions.transformations.concatenate_matrices(target_t, target_R)
                    target_T = tf_conversions.transformations.concatenate_matrices(np.linalg.inv(trans_T), target_T)
                    t_scale, t_shear, self.t_angles, self.t_trans, t_persp = tf_conversions.transformations.decompose_matrix(target_T)
                    t_q=tf_conversions.transformations.quaternion_from_euler(-self.t_angles[0],-self.t_angles[1],-self.t_angles[2])

                    self.TargetTF.transform.translation.x =  self.t_trans[0]
                    self.TargetTF.transform.translation.y = -self.t_trans[1]
                    self.TargetTF.transform.translation.z = -self.t_trans[2]
                    self.TargetTF.transform.rotation.x = t_q[0]
                    self.TargetTF.transform.rotation.y = t_q[1]
                    self.TargetTF.transform.rotation.z = t_q[2]
                    self.TargetTF.transform.rotation.w = t_q[3]
                # 仿真
                else:
                    # self.TargetTF.transform.translation.x = tag.pose.pose.pose.position.x
                    # self.TargetTF.transform.translation.y = tag.pose.pose.pose.position.y
                    # self.TargetTF.transform.translation.z = tag.pose.pose.pose.position.z
                    # self.TargetTF.transform.rotation.x = tag.pose.pose.pose.orientation.x
                    # self.TargetTF.transform.rotation.y = tag.pose.pose.pose.orientation.y
                    # self.TargetTF.transform.rotation.z = tag.pose.pose.pose.orientation.z
                    # self.TargetTF.transform.rotation.w = tag.pose.pose.pose.orientation.w
                    # # TF变换处理
                    # trans_T = transformAsMatrix(self.TargetTF.transform)
                    # target_R = tf_conversions.transformations.quaternion_matrix([0.500, 0.500, -0.500, 0.500])
                    # target_t = tf_conversions.transformations.translation_matrix([0, 0, self.target_dis])
                    # target_T = tf_conversions.transformations.concatenate_matrices(target_t, target_R)
                    # target_T = tf_conversions.transformations.concatenate_matrices(np.linalg.inv(trans_T), target_T)
                    # t_scale, t_shear, self.t_angles, self.t_trans, t_persp = tf_conversions.transformations.decompose_matrix(target_T)
                    # t_q=tf_conversions.transformations.quaternion_from_euler(-self.t_angles[0],-self.t_angles[1],-self.t_angles[2])

                    # self.TargetTF.transform.translation.x = self.t_trans[0]
                    # self.TargetTF.transform.translation.y = -self.t_trans[1]
                    # self.TargetTF.transform.translation.z = -self.t_trans[2]
                    # self.TargetTF.transform.rotation.x = t_q[0]
                    # self.TargetTF.transform.rotation.y = t_q[1]
                    # self.TargetTF.transform.rotation.z = t_q[2]
                    # self.TargetTF.transform.rotation.w = t_q[3]

                    # 转换坐标关系
                    rpy_angles = tf_conversions.transformations.euler_from_quaternion([tag.pose.pose.pose.orientation.x, tag.pose.pose.pose.orientation.y,
                                                                                        tag.pose.pose.pose.orientation.z,tag.pose.pose.pose.orientation.w])
                    rpy_angles = list(rpy_angles)
                    self.TargetTF.transform.translation.x =  tag.pose.pose.pose.position.z - self.target_dis
                    self.TargetTF.transform.translation.y = -tag.pose.pose.pose.position.x
                    self.TargetTF.transform.translation.z = -tag.pose.pose.pose.position.y
                    if(rpy_angles[0]>=0):
                        rpy_angles[0] -= math.pi
                    else:
                        rpy_angles[0] += math.pi
                    rpy_angles[0] *= -1
                    rpy_angles[1] *= -1
                    # 调试用
                    # self.TargetTF.transform.rotation.x = rpy_angles[2]*180/math.pi
                    # self.TargetTF.transform.rotation.y = rpy_angles[0]*180/math.pi
                    # self.TargetTF.transform.rotation.z = rpy_angles[1]*180/math.pi
                    # self.TargetTF.transform.rotation.w = 0
                    wxyz_angles = tf_conversions.transformations.quaternion_from_euler(-rpy_angles[2],rpy_angles[0],rpy_angles[1])
                    self.TargetTF.transform.rotation.x = wxyz_angles[0]
                    self.TargetTF.transform.rotation.y = wxyz_angles[1]
                    self.TargetTF.transform.rotation.z = wxyz_angles[2]
                    self.TargetTF.transform.rotation.w = wxyz_angles[3]
                # 发送的TF接收方直接+即可，不必再考虑负号问题，因为这里已经处理好了。接收方的绝对目标=当前状态+该TF
                self.target_pub.publish(self.TargetTF)

if __name__ == "__main__":
    ros_node = TagDetectNode()
    rospy.spin()  # 开启循环
