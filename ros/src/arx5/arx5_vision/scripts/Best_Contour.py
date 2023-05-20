#!/usr/bin/env python3

# 参数配置
SIM_HSV1_L,SIM_HSV1_H = [64,93,59],[179,255,255]  # 仿真的第1个颜色的HSV参数的下限L和上限H
SIM_HSV2_L,SIM_HSV2_H = [40,111,113],[107,255,255]  # 仿真的第2个颜色的HSV参数的下限L和上限H
REAL_HSV1_L,REAL_HSV1_H = [95,133,85],[179,255,255]  # 实机的第1个颜色的HSV参数的下限L和上限H
REAL_HSV2_L,REAL_HSV2_H = [19,0,105],[60,255,255]  # 实机的第2个颜色的HSV参数的下限L和上限H
# 参考设置
EXPECT_PICK_REF = 426
EXPECT_PLACE_REF = 360


from numpy import ndarray
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import math
import tf_conversions
from typing import Union
import time
import argparse
from arx5_vision.vision import StdVideo


if __name__ == '__main__':

    # 打印一些信息
    def loginfo(msg: str):
        rospy.loginfo("[{}] {}".format(NODE_NAME, msg))
        return

    # 发布ROS命令
    def target_publish(xyz_bias,rotation_angle):
        target = TransformStamped()
        target.header.stamp = rospy.Time.now()
        target.transform.translation.x = xyz_bias[1] # 图像y（前后方向）实际对应机械臂的x轴，dx
        target.transform.translation.y = xyz_bias[0] # 图像x（左右方向）实际对应机械臂的y轴，dy
        target.transform.translation.z = xyz_bias[2] # 图像z（垂直方向）实际对应机械臂的z轴，dz
        # 角度按45度为界进行正逆转动的转换，保证优弧
        if rotation_angle > 45:
            rotation_angle = rotation_angle - 90
        tf_q = tf_conversions.transformations.quaternion_from_euler(0,0,rotation_angle*math.pi/180)  # pick&place状态下，你以为的末端关节的roll，其实是机械臂的yaw
        target.transform.rotation.x = tf_q[0]
        target.transform.rotation.y = tf_q[1]
        target.transform.rotation.z = tf_q[2]
        target.transform.rotation.w = tf_q[3]
        target_pub.publish(target)

    # 图像处理函数
    def image_process(frame:Union[Image,ndarray]):
            # 根据注意力参数进行一些信息是否发送的切换
            global vision_attention
            vision_attention_new = rospy.get_param("/vision_attention", default="pause")
            if args.test != 'no':
                if 'i' in args.test: vision_attention_new =  'pick'+args.test[2]  # 查看识别效果用
                elif 'l' in args.test: vision_attention_new ='place'+args.test[2] # 查看识别效果用
                else: vision_attention_new = args.test  # 比如为‘t’，只要不是p就行，整定HSV阈值用
            if vision_attention_new != vision_attention:
                print("模式切换为：{}".format(vision_attention_new))
                last_vision_attention = vision_attention
                vision_attention = vision_attention_new
            
            # Image图像自动转换为cv2格式
            if isinstance(frame,Image):
                frame  = CvBridge().imgmsg_to_cv2(frame, "bgr8")

            # 若暂停，则直接退出函数，不进行任何图像处理和话题发布
            if vision_attention == 'pause':
                StdVideo.Show('Output',frame,wait_time=1,disable=args.not_show)
                return

            # 颜色阈值确定
            if 'p' not in vision_attention:
                if args.use_real:  # 目前仅针对实机课程有意义
                    if args.test != 'no':
                        StdVideo.color_thresh_determine(device,mode='HSV',show_raw=True)  # 阻塞，按esc退出
                        exit('ESC键按下，视觉程序退出')
                    else:
                        print("当前的HSV阈值为:")  # 提示当前阈值
                        print(f"  1L={HSV_Color['PurpleLR']},1H={HSV_Color['PurpleHR']}")
                        print(f"  2L={HSV_Color['GreenLR']}, 2H={HSV_Color['GreenHR']}")
                        StdVideo.color_thresh_determine(device,mode=vision_attention,show_raw=False)  # 阻塞，按esc退出
                else: StdVideo.color_thresh_determine("camera/color/image_raw",mode=vision_attention)
                rospy.set_param("/vision_attention",'pick0')  # 回到0状态
                vision_attention = 'pick0'

            # 得到二值图
            if ColorFilter:  # 根据奇偶次和pickplace状态选择不同的颜色
                if vision_attention in ['pick0','place1']:  # 一轮中，pick和place利用的颜色不同
                    if not args.use_real:
                        res,binary_img = StdVideo.color_filter(frame,HSV_Color['PurpleLS'],HSV_Color['PurpleHS'])
                    else:  # 实机
                        if vision_attention == 'place1':  # place和pick的参数可以要分开会好一些
                            res,binary_img = StdVideo.color_filter(frame,HSV_Color['PurpleLR'],HSV_Color['PurpleHR'])
                        else: res,binary_img = StdVideo.color_filter(frame,HSV_Color['PurpleLR'],HSV_Color['PurpleHR'])
                else:
                    if not args.use_real:
                        res,binary_img = StdVideo.color_filter(frame,HSV_Color['GreenLS'],HSV_Color['GreenHS'])
                    else:  # 实机
                        if vision_attention == 'place0':
                            res,binary_img = StdVideo.color_filter(frame,HSV_Color['GreenLR'],HSV_Color['GreenHR'])
                        else: res,binary_img = StdVideo.color_filter(frame,HSV_Color['GreenLR'],HSV_Color['GreenHR'])
            # 检测得到边缘图
            if EdgeDetect:
                if not args.use_real:
                    if ColorFilter:
                        binary_img = StdVideo.edge_detect(binary_img,*CannyParam['Sim'],convert=None)
                    else: binary_img = StdVideo.edge_detect(frame,*CannyParam['Sim'])
                else:
                    if ColorFilter:
                        binary_img = StdVideo.edge_detect(binary_img,*CannyParam['Real'],convert=None)
                    else: binary_img = StdVideo.edge_detect(frame,*CannyParam['Real'])
            # 对二值图进行轮廓检测与筛选
            if 'pick' in vision_attention:  # pick模式
                if not args.use_real:
                    center_xy,bias_xyz,rotation_angle,box_int,w_h,cnt = StdVideo.Contour.NFC_F(binary_img,(4000,60000),3.0,(ImageSize[0]/2,expect_pick_y_sim,20000))  # 仿真
                else:  # 实机
                    center_xy,bias_xyz,rotation_angle,box_int,w_h,cnt = StdVideo.Contour.NFC_F(binary_img,(2000,80000),3.0,(ImageSize[0]/2,expect_pick_y_real,20000))  # 实机
                if args.test != 'no': StdVideo.Show('Binary',binary_img,wait_time=1)
            else:  # place模式
                # place模式下mid_y没啥意义,然后‘长宽比’要大些
                if not args.use_real:
                    center_xy,bias_xyz,rotation_angle,box_int,w_h,cnt = StdVideo.Contour.NFC_F(binary_img,(2000,20000),5.5,(ImageSize[0]/2,expect_pick_y_sim,20000))  # 仿真
                else:  # 实机
                    # 裁切图像，避免由于物块大小不均匀造成的下层物块在上层轮廓侧边的漏出误识别（您也可以通过轮廓近似的方式来消除这种凸性缺陷）
                    bi = StdVideo.create_empty_frame(ImageSize[:2],0)
                    bi[:423,:] = binary_img[:423,:]
                    binary_img = bi
                    # 轮廓识别
                    center_xy,bias_xyz,rotation_angle,box_int,w_h,cnt = StdVideo.Contour.NFC_F(binary_img,(1000,20000),5.5,(ImageSize[0]/2,expect_pick_y_real,20000))  # 实机
                if args.test != 'no': StdVideo.Show('Binary',binary_img,wait_time=1)
                if cnt is not None:
                    # 上边沿法（前提是物块上边是近乎平行的，否则不对）（注意摄像头下移则上边沿相对上移）
                    if not args.use_real:
                        reference_y = expect_place_y_sim  # 参考的标准的上边沿的位置（参考值的偏差直接影响了所有物块放置的偏差（即每次放置的系统偏差），是累计偏差的根源，因此一定要尽可能精确确定，并且在放置时可以采用上下轮换逼近的方式避免累计）
                    else: reference_y = expect_place_y_real
                    upper_edge = center_xy[1] - w_h[0]/2  # 注意此时减的是宽，因为宽更小，符合实际情况
                    if not args.negative:
                        bias_xyz[0] *= -1  # x方向偏差照旧，只是另一头方向要反一下
                        bias_xyz[1] = upper_edge - reference_y # 将y方向的偏差改为物块上边沿的位置(物块在)
                    else: bias_xyz[1] = reference_y - upper_edge # 将y方向的偏差改为物块上边沿的位置                  
                    # 不管rotation（因为不旋转才是正确的搭建姿态）
                    rotation_angle = 0

            # 若成功筛选出目标
            if cnt is not None:
                bias_xyz[2] = 0  # 不考虑z方向偏差
                StdVideo.Contour.draw(frame,[box_int])
                StdVideo.Draw.basic_draw(frame,(640,360),3,(0,0,255),-1,tips='circle')  # 绘制图像中心点（-1表示填充）
                target_publish(bias_xyz,rotation_angle)
                if args.test != 'no':
                    loginfo("偏差量为：x:{:.1f} y:{:.1f} z:{:.1f} yaw:{:.1f} ".format(bias_xyz[1],bias_xyz[0],0,rotation_angle))
                    loginfo(f'中心坐标为：{center_xy}，w_h为：{w_h}')
                    loginfo(f'上边沿为：{center_xy[1] - w_h[0]/2}')
            # 图像展示
            StdVideo.Show('Output',frame,wait_time=1,disable=args.not_show)

    """ ***********识别参数配置************ """
    # 颜色阈值
    HSV_Color = {
        # 仿真
        'PurpleLS':[64,93,59],   'PurpleHS':[179,255,255],
        'GreenLS':[40,111,113],  'GreenHS':[107,255,255],
        # 实机
        # 'PurpleLR':[90,180,70], 'PurpleHR':[178,255,255], # 2.10+2.25晚均可
        # 'PurpleLR':[70,70,24], 'PurpleHR':[152,255,255],  # 2.26下午16点
        # 'PurpleLR':[90,174,0], 'PurpleHR':[179,255,255],  # 3.2晚
        # 'PurpleLR':[67,91,23], 'PurpleHR':[179,255,255],
        'PurpleLR':[90,125,0], 'PurpleHR':[179,255,255],  # 4.13土木

        # 'GreenLR':[18,97,39],  'GreenHR':[93,255,255],  # 2.10
        # 'GreenLR':[47,118,53],  'GreenHR':[85,252,161], # 2.25晚891
        # 'GreenLR':[47,150,20],  'GreenHR':[77,255,254], # 4.10土木
        'GreenLR':[42,140,60],  'GreenHR':[84,255,254], # 4.13土木
        # 'GreenLR':[47,171,18],  'GreenHR':[70,255,71],  # 2.26下午16点
        # 'GreenLR':[37,111,24],  'GreenHR':[73,255,255],   # 3.26下午1点半

        'PurpleLU':[67,91,23], 'PurpleHU':[179,255,255],
        'GreenLU':[37,111,24],  'GreenHU':[73,255,255],
    }
    # 期望位置
    expect_pick_y_sim = 529
    expect_place_y_sim = 456
    expect_pick_y_real = EXPECT_PICK_REF  # 可以以这个为基准
    expect_place_y_real = EXPECT_PLACE_REF

    """ 算法相关 """
    CannyParam = {'Sim':[210,240,5,True],'Real':[0,0,0,False]}
    ColorFilter = True
    EdgeDetect = False

    """ ***********节点初始化************ """
    NODE_NAME = 'arx5_cube_detect'
    rospy.init_node(NODE_NAME)
    loginfo("Start {} node.".format(NODE_NAME))
    parser = argparse.ArgumentParser("ARX PickPlace")
    parser.add_argument('-r','--use_real',action='store_true',help='use sim or real robot')
    parser.add_argument('-ns','--not_show',action='store_true',help='show img or not')
    parser.add_argument('-t','--test',type=str, help='test mode: pi0 pi1 pl0 pl1',default='no')
    parser.add_argument('-ng','--negative',action='store_true',help='negative place')
    parser.add_argument('-hsv','--hsv_params',type=str, help='example: 1,2,3;4,5,6',default='')
    args, unknown = parser.parse_known_args()

    # # 使用hsv参数替换后的参数
    if args.use_real:
        HSV_Color['PurpleLR'] = REAL_HSV1_L
        HSV_Color['PurpleHR'] = REAL_HSV1_H
        HSV_Color['GreenLR']  = REAL_HSV2_L
        HSV_Color['GreenHR']  = REAL_HSV2_H
    elif args.hsv_params != '':
        HSV_Color['PurpleLS'] = SIM_HSV1_L
        HSV_Color['PurpleHS'] = SIM_HSV1_H
        HSV_Color['GreenLS']  = SIM_HSV2_L
        HSV_Color['GreenHS']  = SIM_HSV2_H

    target_pub = rospy.Publisher("/target_TF",TransformStamped,queue_size=1) # queue_size=1表明只发布最新数据
    vision_attention = 'pause'  # 初始默认为pause
    print("模式初始为：{}".format(vision_attention))
    if args.not_show: print('图像显示设置为：不显示图像')
    # 接收相机数据并进行处理
    if not args.use_real:
        """ 获取相机参数，主要是用到了宽和高来作为参考"""
        camera_info_topic = rospy.get_param("~camera_info", default="/camera/color/camera_info")
        try:
            camrera_info:CameraInfo = rospy.wait_for_message(camera_info_topic,CameraInfo,timeout=2)
            ImageSize = [camrera_info.width,camrera_info.height]; print('图像大小为：',ImageSize)
        except: exit("获取仿真相机信息失败，程序退出")
        """ 启动图片订阅 """
        rospy.Subscriber("camera/color/image_raw",Image,image_process,queue_size=1)
        rospy.spin()
    else:  # 实机
        device = 2
        # device = max(StdVideo.Find(max_num=3))
        cap = StdVideo.Cap(2)
        ImageSize = [int(StdVideo.Info(device,StdVideo.Types.CAP_FRAME_WIDTH)),int(StdVideo.Info(device,StdVideo.Types.CAP_FRAME_HEIGHT))]
        print('图像大小为：',ImageSize)
        print('等待视频流稳定......')
        time.sleep(2)
        print('开始获取图像')
        while True:
            frame = StdVideo.Read(device)
            image_process(frame)
            StdVideo.ros_spin_once(0.001)