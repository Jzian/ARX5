import cv2
import numpy as np
from numpy import ndarray
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import math,os
from typing import Optional,Union,List,Tuple,Dict,Any
import time
from threading import Thread
import queue
from matplotlib import pyplot as plt
import atexit


class StdVideo(object):  # 推荐：import StdVideo as sv
    """
        从视频图像获取到处理后图像信息的输出一条龙的CV类，底层主要基于OpenCV实现，在其上进行了进一步封装以求更加方便使用。
        主要用于直接从视频设备获取图像进行处理；同时也对ROS提供一定支持。
    """
    windows_set = set()  # 维护不同的windows窗口(尽管窗口可以和frame建立一种对应关系，但是frame是一直变化的，从而维护这种联系意义不大，故采用集合变量)
    cap_dict = {}  # 维护不同的视频流(索引号-cap实例)
    video_info = {}  # 维护不同的视频流的信息（索引号-视频信息）
    actual_fps = {};__fps_cal = {}  # 维护不同窗口的实际帧率信息（窗口名-帧率）
    pub_topics = {}  # 维护不同的图片发布话题（话题名-消息类型）
    cap_buffer:Dict[Any,queue.Queue] = {}  # 图片队列

    class Types(object):
        """ 定义一些名称便于使用(所有其它子类中的类型也均统一放到这里) """
        COLOR_BGR2GRAY = cv2.COLOR_BGR2GRAY
        COLOR_BGR2HSV = cv2.COLOR_BGR2HSV
        COLOR_BGR2RGB = cv2.COLOR_BGR2RGB
        COLOR_BGR2LAB = cv2.COLOR_BGR2LAB
        COLOR_BGR2YCrCb = cv2.COLOR_BGR2YCrCb

        THRESH_BINARY = cv2.THRESH_BINARY  # 超过阈值时取maxval，否则取0（常用）
        THRESH_BINARY_INV = cv2.THRESH_BINARY_INV  # 与cv2.THRESH_BINARY相反
        THRESH_TRUNC = cv2.THRESH_TRUNC  # 超过阈值时取阈值，否则不变（阈值处截断）
        THRESH_TOZERO = cv2.THRESH_TOZERO  # 超过阈值时不变，否则取0（低于阈值取0）
        THRESH_TOZERO_INV = cv2.THRESH_TOZERO_INV  # 超过阈值时取0，否则不变（与cv2.THRESH_TOZERO相反）
        THRESH_ADAPTIVE_MEAN_C  = cv2.ADAPTIVE_THRESH_MEAN_C  # 为局部邻域块的平均值，该算法是先求出块中的均值。
        THRESH_ADAPTIVE_GAUSSIAN_C = cv2.ADAPTIVE_THRESH_GAUSSIAN_C  # 为局部邻域块的高斯加权和。该算法是在区域中(x, y)周围的像素根据高斯函数按照他们离中心点的距离进行加权计算。

        WINDOW_NORMAL = cv2.WINDOW_NORMAL        # 用户能够调节窗口大小（常用）
        WINDOW_AUTOSIZE = cv2.WINDOW_AUTOSIZE    # 根据图像大小显示窗口，大小固定
        WINDOW_FREERATIO = cv2.WINDOW_FREERATIO  # 调整图像，不考虑其比例
        WINDOW_KEEPRATIO = cv2.WINDOW_KEEPRATIO	 # 调整图像，保持图像比例

        CAP_TIME_LAST = cv2.CAP_PROP_POS_MSEC
        CAP_NOW_FRAME =  cv2.CAP_PROP_POS_FRAMES
        CAP_FRAME_WIDTH = cv2.CAP_PROP_FRAME_WIDTH
        CAP_FRAME_HEIGHT = cv2.CAP_PROP_FRAME_HEIGHT
        CAP_FPS  = cv2.CAP_PROP_FPS
        CAP_PROP_FOURCC = cv2.CAP_PROP_FOURCC  # 视频格式，一般为MJPG和YUYV

        CONTOUR_RETR_EXTERNAL = cv2.RETR_EXTERNAL        # 只检测外轮廓（常用）
        CONTOUR_RETR_LIST = cv2.RETR_LIST                # 检测的轮廓不建立等级关系
        CONTOUR_RETR_CCOMP = cv2.RETR_CCOMP              # 建立两个等级的轮廓，上面的一层为外边界，里面的一层为内孔的边界信息。如果内孔内还有一个连通物体，这个物体的边界也在顶层
        CONTOUR_RETR_TREE = cv2.RETR_TREE                # 建立一个等级树结构的轮廓
        CONTOUR_APPROX_NONE = cv2.CHAIN_APPROX_NONE      # 存储所有的轮廓点，相邻的两个点的像素位置差不超过1
        CONTOUR_APPROX_SIMPLE = cv2.CHAIN_APPROX_SIMPLE  # 压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标

        DRAW_FONT_SIMPLEX  = cv2.FONT_HERSHEY_SIMPLEX  # 各字体效果：https://cloud.tencent.com/developer/article/1821937
        DRAW_FONT_PLAIN = cv2.FONT_HERSHEY_PLAIN
        DRAW_FONT_DUPLEX = cv2.FONT_HERSHEY_DUPLEX
        DRAW_FONT_COMPLEX = cv2.FONT_HERSHEY_COMPLEX
        DRAW_FONT_TRIPLEX = cv2.FONT_HERSHEY_TRIPLEX
        DRAW_FONT_COMPLEX_SMALL = cv2.FONT_HERSHEY_COMPLEX_SMALL
        DRAW_FONT_SCRIPT_SIMPLEX = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
        DRAW_FONT_SCRIPT_COMPLEX = cv2.FONT_HERSHEY_SCRIPT_COMPLEX

    @classmethod
    def Find(cls,max_num=5):
        """ 帮助搜寻所有可以使用的设备号 """
        device_avilable = []
        for i in range(max_num):
            try: cls.Cap(i)
            except:pass
            else:
                device_avilable.append(i)
                # TODO:退出刚才cap的内容
        print(f"在搜寻区间[0,{max_num-1}]中，可用的设备号共有：",device_avilable)

    @classmethod
    def Cap(cls,device=0,wh_set:tuple=None,fps_set:int=None,format=None,api=cv2.CAP_V4L2,buffer=0):
        """
            打开一个设备视频流。
            要捕获视频，你需要的参数可以是设备device索引或视频文件的名称(唯一身份ID)。设备索引就是指定哪个摄像头的数字。通常仅有一个摄像头会被连接。
            可以简单地传0(或-1)。多个设备时，一般会按照上电顺序确定索引。可以传max或none，此时将自动寻找当前序号最大的设备进行开启。
            format为None则不更改相机视频输出格式，另外可以改成‘MJPG’和‘YUYV’，从而指定格式。
            api默认为Linux下最好的cv2.CAP_V4L2，若是Windows，请更换为默认。
        """
        print('视频流开启中......')
        if device in ['max',None]:
            device = max(cls.Find(max_num=5))
        if cls.cap_dict.get(device) is not None:
            raise Exception('请不要重复获取同一设备的视频流')
        else: cap = cv2.VideoCapture(device,api)  # Linux下cv2.CAP_V4L2是最佳的视频接口（理论上默认安装了已经）
        if not cap.grab():  # 由于isopened函数始终返回none，因此改用read来判断是否开启
            raise Exception(f'Unable to open device: {device}')
        # 设置视频宽和高
        if wh_set is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, wh_set[0])  # 设置图像宽度
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, wh_set[1]) # 设置图像高度
        # 设置视频帧率
        if fps_set is not None:
            cap.set(cv2.CAP_PROP_FPS,fps_set)   # 设置帧率
        # 设置图像格式
        if format is not None: cap.set(6,cv2.VideoWriter.fourcc(*format))
        cls.cap_dict[device] = cap
        # 记录视频主要参数：尺寸、帧率
        fps = cap.get(cv2.CAP_PROP_FPS)
        cls.video_info[device] = {'width':cap.get(cv2.CAP_PROP_FRAME_WIDTH),'height':cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                    'fps':fps}
        if buffer>0:  # 创建图片缓冲，即开启一个线程，以相机的帧率为频率进行图像的读取存储。
            cls.cap_buffer[device] = queue.Queue(maxsize=buffer)
            def buffer_read():
                period_half = 1/fps/2
                while True:
                    # 从-1到0，先进先出，即0位始终为最旧，-1为最新
                    cls.cap_buffer[device].put(cap.read())
                    time.sleep(period_half)
            Thread(target=buffer_read,daemon=True).start()
        # 注册程序退出时对cap和所有windows的关闭函数
        atexit.register(cap.release)
        atexit.register(cv2.destroyAllWindows)
        print('视频流成功开启！')
        return cap

    @classmethod
    def Read(cls,device=0,resize_xyc:Optional[tuple]=None,color_convert=None,return_raw=False):
        """
            从视频流中获取一帧图像,返回处理后的图像和原图像。
            纯cap视频流没有缓冲区，因此read始终读取的是最新的一帧。
            resize_xyc可以是两个元素也可以三个元素且第三个元素为0，表示将图像转为灰度图，第三个元素大于0则表示按照该阈值对图像进行常用的二值化操作。
                前两个元素可以均为none，此时仅最后一个元素将生效。
                考虑简洁性，不支持单独仅某个方向的resize，请根据图像实际大小自行将不需调整的方向设置为原始大小。
        """
        if cls.cap_dict.get(device) is None: cls.Cap(device)  # 若未检测到视频流，则创建，从而起始不用先cap后read
        # 是否从buffer中获取
        if cls.cap_buffer.get(device) is None:
            ret, frame = cls.cap_dict[device].read()  # ret为true为正常读取到贞，false表示读取到视频的结尾。对于开启相机的正常情况下始终为true
        else:
            ret, frame = cls.cap_buffer[device].get(timeout=0.2)
        # 判断是否读取到非空图像
        if frame is None: exit('Unable to read image from target device')
        elif not ret: exit('The video has been read over')
        frame_cp = frame.copy()  # 复制防止改动原图
        if resize_xyc is not None:
            lis2 = resize_xyc[:2]
            if None not in lis2:
                frame_cp = cls.Geometry.Resize(frame_cp,lis2)
            if len(resize_xyc)==3:
                if resize_xyc[2] == 0: frame_cp = cv2.cvtColor(frame_cp,cls.Types.COLOR_BGR2GRAY)
                else: frame_cp = cls.Threshold.gray_global(frame_cp,resize_xyc[2],255)
        if color_convert is not None:
            frame_cp = cv2.cvtColor(frame_cp,color_convert)
        # print(cls.cap_dict[device].get(cv2.CAP_PROP_POS_FRAMES))
        if return_raw:
            return frame_cp,frame
        else: return frame_cp

    @classmethod
    def Jump(cls,device,target_frame=-1,inc=False,not_pass=0):
        """
            跳到某一帧。默认为最新帧即不跳帧。相比于Read更快。
            inc为true时target_frame表示目标帧为当前帧增量。
            not_pass为0时目标帧实际会pass掉，下次read的是目标帧的下一帧。
        """
        if inc: target_frame = cls.cap_dict[device].get(cv2.CAP_PROP_POS_FRAMES) + target_frame
        while cls.cap_dict[device].get(cv2.CAP_PROP_POS_FRAMES)<target_frame-not_pass:  # 当前帧，基于以0开始的被捕获或解码的帧索引
            cls.cap_dict[device].grab()

    @classmethod
    def Info(cls,device,info_name):
        """ 除了基本信息的全局字典外，提供获取其它参数的函数 """
        if info_name == cls.Types.CAP_PROP_FOURCC:
            fourcc = int(cls.cap_dict[device].get(cv2.CAP_PROP_FOURCC))
            fourcc_str = chr(fourcc & 0xFF) + chr((fourcc >> 8) & 0xFF) + chr((fourcc >> 16) & 0xFF) + chr((fourcc >> 24) & 0xFF)
            return fourcc_str
        else: return cls.cap_dict[device].get(info_name)

    @classmethod
    def Show(cls,window_name:str,frames=None,window_size=(640,480),window_mode=cv2.WINDOW_NORMAL,wait_time=1,rows=1,cols=None,EXIT_KEY:Union[str,int]=27,show_fps=False,cal_fps=False,func_bound=(),auto3D=True,use_plt=False,disable=False):
        """
            显示图像(融合了窗口配置、图片拼接与显示、帧率提示、按键检测、绑定函数等功能)；图像为none时将自动完成读取，因此实现了一行代码读取和显示图像
            可以实现多幅图像的拼接显示（多幅图像放到tuple中），将按照给定的rows,cols参数进行排列显示
            auto3D=True时自动将单通道图像转换为3通道，从而可以与3通道图像一同显示
            disable=True时该函数将失效，图片不再进行展示
            EXIT_KEY默认为27，表示按下ESC键退出程序，可以改成其它键值或字符
            show_fps和cal_fps与帧率有关，前者为True时将计算并在图像左下角实时显示帧率，而前者为False后者为True时将计算帧率并保存在类变量中，可供取用
            默认创建窗口只在该函数中才最终生效，设定window_mode为none才认为"窗口已经之前设定好了"，不新设定窗口，不过仍然可以调整窗口大小
            func_bound可以是一个或者多个函数对象，用来绑定一些窗口功能，如鼠标位置打印等，绑定函数的参数固定为window_name,frames（即窗口名和图像），示例见print_mouse_color方法
            use_plt=True将使用matplotlib.pyplot进行图片展示，不推荐
        """
        if not disable:
            # 甚至集成到了如果frames为none，则直接自动read
            if frames is None: frames = cls.Read()
            # 拼接图片(图片数量为1则返回原图)
            if not use_plt:
                frames = cls.Combine(frames,rows,cols,auto3D)
            try: cv2.getWindowProperty(window_name,0)  # 使用这种方式进行判断窗口是否存在，耗时在微妙级，不影响性能
            except: cls.windows_set.discard(window_name)  # 如果不存在，则执行一次窗口集合移除操作，便于重新进行配置，消除窗口关闭后大小改变的bug
            # 初次使用定义窗口（或窗口摧毁后重新定义）
            if window_name not in cls.windows_set:  # 仅在该函数中配置全局窗口set
                if window_mode is not None:
                    cls.windows_set.add(window_name)
                    try: cv2.destroyWindow(window_name)  # 为避免之前临时创建了window这里要尝试清除一下否则后面的设定不会生效
                    except:pass
                    cv2.namedWindow(window_name, window_mode)
                if window_size is not None and window_mode!=cv2.WINDOW_AUTOSIZE:
                    cv2.resizeWindow(window_name,*window_size)
                cls.__fps_cal[window_name] = [0,0]  # 第一个负责计数控制，第二个负责记录时间；可用于计算每个窗口的图像实际显示帧率（第一帧忽略，然后从第一帧show结束(相当于第二帧开始)到到下一帧show前）
            # 执行绑定的函数（主要是一些窗口回调函数）
            if callable(func_bound): func_bound(window_name,frames)
            elif len(func_bound)>0:
                for func in func_bound:
                    func(window_name,frames)
            # 性能计算：结束
            if (cal_fps or show_fps) and cls.__fps_cal[window_name][0]==1:
                cls.__fps_cal[window_name][0]=0
                now_time = time.time()
                cls.actual_fps[window_name] = 1/(now_time - cls.__fps_cal[window_name][1])
                if show_fps:  # 在图像左下角显示fps
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(frames, f'FPS:{cls.actual_fps[window_name]:.2f}',
                                (5,frames.shape[0]-5),font,1,(255,255,255),1,cv2.LINE_AA)  # 图片、文字、坐标（文字左下角）、类型、大小、颜色、厚度、线条类型
            # 图片显示与按键退出
            if not use_plt: cv2.imshow(window_name,frames)
            else: cls.plt_imgs_show(frames)  # 使用爬plt进行图片显示
            k = cv2.waitKey(wait_time) & 0xFF  # 如果使用的是64位计算机，则必须&0XFF或&0xff
            if isinstance(EXIT_KEY,str):
                if k == ord(EXIT_KEY):
                    if wait_time > 0: raise Exception(f'{EXIT_KEY}键按下，程序结束')
                    else: cv2.destroyWindow(window_name)
            elif k == EXIT_KEY:
                if wait_time > 0: raise Exception(f'ESC键按下，程序结束')
                else: cv2.destroyWindow(window_name)
            # 性能计算：开始
            if (cal_fps or show_fps) and cls.__fps_cal[window_name][0] == 0:
                cls.__fps_cal[window_name][1] = time.time()
                cls.__fps_cal[window_name][0] = 1

    @classmethod
    def Combine(cls,frames:Union[ndarray,List[ndarray],tuple],rows=1,cols=None,auto3D=True):
        """
        指定行数或列数拼为基准来接图片；均指定则以列数为准；
        可设置auto3D为false,则将不会自动检测维度并进行调整显示；（默认自动检测，因为单次检测耗时不多在微妙级）
        """
        # 根据列数或行数将多幅图像拼接
        if not isinstance(frames,ndarray):
            num = len(frames)
            if auto3D:  # 二维图自动转为3D图
                for frame in frames:
                    if frame.ndim == 2: frame = cls.To3D(frame)
        else: num = 1
        if num > 1:
            row_frames = []
            if cols is not None:  # 按列排序
                h_num = num//cols  # 求商,得到共有多少行
                res_v = num%cols  # 求余，得到最后一行剩下几个空位
                i=0
                for i in range(h_num):
                    row_frames.append(np.hstack(frames[i*cols:(i+1)*cols]))
                if h_num>0: i+=1
                if res_v > 0:
                    black = frames[0]*0
                    row_frames.append(np.hstack(frames[i*cols:]+[black for _ in range(res_v)]))
                # 得到最终拼接好的图像
                frames = np.vstack(row_frames)
            # 默认按行排序
            else:
                rem = num%rows  # 求出余数reminder
                if rem > 0:
                    res = rows-rem  # 需要补充的数量
                    cols = int(num/rows) + 1  # 每行列数
                    black = frames[0]*0
                    frames.append([black for _ in range(res)])  # 用黑色图补充缺少的图片
                else: cols = int(num/rows)
                # 行拼接
                for i in range(rows):
                    row_frames.append(np.hstack(frames[i*cols:(i+1)*cols]))
                # 列拼接
                frames = np.vstack(row_frames)
        return frames

    @classmethod
    def Save(cls,frame:ndarray,name='Image',path="../Images/",cover:int=0):
        """ 保存图像(该函数通过启动线程的方式进行图像与视频保存，对主进程影响较小) """
        if cover>0:  # 不覆盖，修改下name
            if not hasattr(cls.Save,name): cls.Save.__dict__[name] = 0
            name+=f'_{cls.Save.name[0]}'
            cls.Save.__dict__[name]+=1
            if cls.Save.__dict__[name] >= cover:
                cls.Save.__dict__[name]=0
        # 开启线程进行图片写入（实测是线程安全的，不论是否覆盖）
        Thread(target=cv2.imwrite,args=(path+name,frame),daemon=True).start()

    @staticmethod
    def To3D(frame:ndarray)->ndarray:
        """ 将灰度或二值图变成3D图，从而可以跟color图一起通过imshow展示 """
        return np.dstack((frame,frame,frame))  # 为了实现并排显示，需要将二值图变成3d图

    @classmethod
    def ToROS(cls,frame:ndarray,name='',data_class=None):
        """ 选择发送转换后的图片(当name不为none时)或仅将图片返回 """
        ros_img = CvBridge().cv2_to_imgmsg(frame)
        if name != '' and data_class is not None:
            if not cls.pub_topics.get(name):  # 首次处理进行发布者创建
                img_pub = rospy.Publisher(name,data_class)
                cls.pub_topics[name]=img_pub
            # 话题发布
            cls.pub_topics[name].publish(ros_img)
        else: return ros_img

    @staticmethod
    def ToCV2(frame)->ndarray:
        """ 将ROS图像或realsense彩色图像转为OpenCV格式 """
        frame_type = type(frame)
        if frame_type is Image:
            return CvBridge().imgmsg_to_cv2(frame)
        else:
            import pyrealsense2 as rs
            if frame_type is rs.pyrealsense2.BufData:
                return np.asanyarray(frame)
            elif frame_type is rs.pyrealsense2.video_frame:
                return np.asanyarray(frame.get_data())

    @classmethod
    def ToSerial(cls,data_cmd,port_vpid:str,baudrate=115200,bytesize=8,parity='N',stopbits=1,timeout=None,find=False):
        """
        通过串口发送有关数据；port_pid可以是端口号（Linux下比如是'/dev/ttyUSB0',Windows下比如是'COM8'），也可以是设备的vid和pid信息结合。如，
        vid为6790，PID为29987，则写为'6790_29987'。
        """
        if not hasattr(cls.ToSerial,f'{port_vpid}'):
            from serial import Serial  # 要装pyserial
            import serial.tools.list_ports
            if find is True:
                port_list = list(serial.tools.list_ports.comports())
                device_list = [port.device for port in port_list]
                vid_list = [port.vid for port in port_list]
                pid_list = [port.pid for port in port_list]
                description_list = [port.description for port in port_list]
                print('共检测到这些设备：',device_list)
                print('设备的VID为：',vid_list)
                print('设备的PID为：',pid_list)
                print('设备的descriptio为：',description_list)
                print(type(pid_list[0]))
                return
            cls.ToSerial.__dict__[f'{port_vpid}'] = None
            #继承串口类
            class MySerial(Serial):
                """
                继承串口类，实现了可以在初始化时不用配置任何信息，通过新增加的comset函数进行配置，然后通过connect进行鲁棒的连接。
                初始化和comset可以都不指定端口号，而是在connect时再指定。
                """
                # 配置串口参数（可以不包括端口号）
                def ComSet(self, baudrate, bytesize, parity, stopbits, timeout,port=None):
                    d={'baudrate': baudrate, 'bytesize': bytesize, 'parity':parity, 'stopbits':stopbits, 'timeout': timeout}
                    self.apply_settings(d)
                    self.port = port
                # 选择端口号并尝试连接
                def Connect(self,rs_port=None):
                    if self.is_open:
                        print(f'已成功连接串口{self.port}')
                        return
                    if rs_port is None:
                        if self.port is not None:
                            rs_port = self.port
                        else: raise Exception('缺少串口号，请检查串口配置')
                    # 没连接到串口则一直连接
                    while True:
                        port_list = list(serial.tools.list_ports.comports())
                        if len(port_list) == 0: exit("没有可用串口")
                        else:
                            port_list = [port.device for port in port_list]
                            port_list = ";".join(list(map(str,port_list)))  # 将字符串列表转换为一整个字符串
                            if rs_port in port_list:  # 判断选用的端口是否在其中
                                self.port = rs_port
                                self.open() # 开启串口
                                if self.is_open:
                                    print(f"成功连接串口{rs_port}")
                                    return
                                else: print(f"已找到目标端口号{rs_port}，但无法连接")
                            else: print("未找到目标端口号{}。找到的所有端口为：{}".format(rs_port,port_list))
            # 串口初始化与连接
            if '_' in port_vpid:
                vid, pid = port_vpid.split('_')
                port_list = list(serial.tools.list_ports.comports())
                device_list = [port.device for port in port_list]
                vid_list = [port.vid for port in port_list]
                pid_list = [port.pid for port in port_list]
                if (int(pid) in pid_list) and (int(vid) in vid_list):
                    port_vpid = device_list[pid_list.index(int(pid))]
                else: exit('错误：pid和vid与现连接所有设备不匹配')
            ser = MySerial(port_vpid,baudrate,bytesize,parity,stopbits,timeout)
            # ser.Connect()
            cls.ToSerial.__dict__[f'{port_vpid}'] = ser
            atexit.register(ser.close)  # 程序退出时关闭串口
        if data_cmd is not None:  # 发送数据
            cls.ToSerial.__dict__[f'{port_vpid}'].write(data_cmd)

    @staticmethod
    def ImageRead(path:str):
        return cv2.imread(path)

    @staticmethod
    def Cover(base:ndarray,coveror:ndarray,position=(0,0),mask=None,quit=True):
        """
            将coveror覆盖到base上,position为左上角（因为OpenCV图像的左上角是图像原点） 
            若coveror右下角超出了base，quit为真多余的部分将被忽略，否则将自动调整图像大小刚好覆盖base。
            mask为none，则直接覆盖，否则根据mask提取出coveror的特定区域进行对应的覆盖。mask可参考如下方式获得：
                coverorgray = cv2.cv2tColor(coveror,cv2.COLOR_BGR2GRAY)
                ret, mask = cv2.threshold(coverorgray, 10, 255, cv2.THRESH_BINARY)
        """
        # 我想把logo放在左上角，所以我创建了ROI
        row_end = position[1] + coveror.shape[0]
        col_end = position[0] + coveror.shape[1]
        if row_end > base.shape[0] or col_end > base.shape[1]:
            if quit:  # 裁剪
                coveror = coveror[:base.shape[0]-position[1],:base.shape[1]-position[0]]
            else:  # 放缩
                coveror = cv2.resize(coveror,(base.shape[0]-position[1],base.shape[1]-position[0]))
        # 提取出base的roi位置的局部图
        roi = base[position[1]:row_end, position[0]:col_end]
        if mask is None: mask = cv2.add(coveror,255)  # 加了255必然全变成了最大值，即认为全部为mask
        # 根据logo的掩码创建其相反掩码
        mask_inv = cv2.bitwise_not(mask)
        # 现在将ROI中logo的区域涂黑
        base_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
        # 仅从logo图像中提取logo区域
        coveror_fg = cv2.bitwise_and(coveror,coveror,mask = mask)
        # 将logo放入ROI并修改主图像
        dst = cv2.add(base_bg,coveror_fg)
        base[position[1]:row_end, position[0]:col_end] = dst
        return base

    @classmethod
    def plt_imgs_show(cls,frames,row_col=None,new_thread=False,daemon=True,convert_color=True,dpi=100):
        """
            通过plt展示多图是方便的，但是也就因此无法使用OpenCV的一些额外的功能，如窗口回调、waitkey等等;
            若row_col为none，则将采用尽可能的正方形方式对图像进行恰当的显示；
            convert_color为true则对图像进行bgr到rgb的转换，若已经转换了，请设为false；
            new_thread为true时让图片显示可以在另外的线程中运行，从而不会阻塞当前线程。此时daemon表明
        """
        if new_thread:
            Thread(target=cls.plt_imgs_show,daemon=daemon,args=(frames,row_col,False,convert_color)).start()
            return
        if row_col is None:
            if isinstance(frames,ndarray):
                row_col = (1,1)
                frames = [frames]
                show_title = False
            else:
                show_title = True
                len_ = len(frames)
                len_sqr = math.sqrt(len_)
                len_sqr_int = int(len_sqr)
                if len_sqr_int * (len_sqr_int+1) < len_:
                    row_col = (len_sqr_int+1,len_sqr_int+1)
                else: row_col = (len_sqr_int+1,len_sqr_int)
        plt.figure(dpi=dpi)  # 调整图像dpi，dpi过低会无法显示一些图像细节信息
        for index,frame in enumerate(frames,1):
            if convert_color: frame = cv2.cvtColor(frame,cls.Types.COLOR_BGR2RGB)
            plt.subplot(*row_col,index),plt.imshow(frame,'gray'),plt.axis('off')
            plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)  # 减小白色边框
            if show_title: plt.title(f'{index}')
        plt.show()

    @classmethod
    def print_mouse_position(cls,window_name,click=False,enable=True):
        """ click为true则点击后才打印当前坐标信息，否则时刻刷新打印 """
        if enable and not hasattr(cls.print_mouse_position,window_name):
            cls.print_mouse_position.__dict__[window_name] = True
            def call_back(event,x,y,flags,param):  # 创建鼠标回调函数具有特定的格式，该格式在所有地方都相同。
                if click and event == cv2.EVENT_LBUTTONDOWN:
                    print(x, y)
                else: print(x, y)  # 始终打印
            cv2.setMouseCallback(window_name,call_back)

    @classmethod
    def print_mouse_color(cls,window_name=None,frame=None,print_pos=True):
        """  """
        def bind_to(window_name,frame):
            """ 所有的绑定函数都有相同的参数：第一个是窗口名，第二个是frame """
            if not hasattr(bind_to,window_name):  # 同一个窗口仅执行一次回调函数设置
                bind_to.__dict__[window_name] = frame
                if window_name not in StdVideo.windows_set:  # 为保证setMouseCallback不出错，先创建一个空窗口
                    cv2.namedWindow(window_name)
                    # TODO: 问题：call_back返回的是窗口的位置，而不是图片的位置，因此，如果窗口大小和这里的图片大小不一致就会对应不上
                def call_back(event,x,y,flags,param):
                    if event == cv2.EVENT_LBUTTONDOWN:
                        # 根据维度判断颜色是灰度还是rgb
                        if bind_to.__dict__[window_name].ndim == 2:
                            if print_pos:
                                print('({},{})点灰度值为：{}'.format(x,y,bind_to.__dict__[window_name].item(y,x)))
                            else:
                                print('该点灰度值为：{}'.format(x,y,bind_to.__dict__[window_name].item(y,x)))  # item方法比直接取值更好
                        else:
                            print('({},{})点BGR值为：{}'.format(x,y,bind_to.__dict__[window_name][y,x]))
                            # 关于numpy的一些注意：Numpy是用于快速数组计算的优化库。因此，简单地访问每个像素值并对其进行修改将非常缓慢，因此不建议使用
                            # 对于单个像素访问，Numpy数组方法array.item()和array.itemset())被认为更好，但是它们始终返回标量
                            # 如果要访问所有B，G，R值，则需要分别调用所有的array.item()。距离：img.itemset((10,10,2),100)
                            # 这里对性能无要求，因此直接取值更方便
                cv2.setMouseCallback(window_name,call_back)
            else: bind_to.__dict__[window_name] = frame
        # 立刻执行绑定
        if None not in (window_name,frame): bind_to(window_name,frame)
        # 返回绑定函数
        else: return bind_to

    @staticmethod
    def color_filter(frame:ndarray, ColorL:list, ColorH:list,convert=cv2.COLOR_BGR2HSV):
        """ 根据HSV选择合适的颜色阈值对彩色图像（默认为BGR格式）进行处理 """
        frame_cp = frame.copy()
        cvt = cv2.cvtColor(frame_cp,convert)
        lower_thresh = np.array(ColorL)
        upper_thresh = np.array(ColorH)
        mask = cv2.inRange(cvt, lower_thresh, upper_thresh)
        res = cv2.bitwise_and(frame_cp, frame_cp, mask=mask)
        return res, mask

    @staticmethod
    def color_exchange(frame:ndarray,order=(1,0,2)):
        """ order=(1,0,2)意为，原来图像的通道1变为通道0，通道0变为通道1，通道2不变，依次类推 """
        bgr = (frame[:,:,0],frame[:,:,1],frame[:,:,2])
        lis = [0,0,0]
        for index,ele in enumerate(order):
            lis[index] = bgr[ele]
        return cv2.merge(lis)

    @classmethod
    def color_thresh_determine(cls,frame_dev_topic=None,default=0,mode='HSV',save_path=None,show_raw=False):
        """  # TODO:增加腐蚀膨胀操作的滑条
            确定最佳HSV阈值的函数：frame可以为图像（ndarray）、设备号（int）或ros话题名（str）
            frame为none则尝试默认参数进行图片获取：先尝试设备0，再尝试获取ros消息，都不行则报错退出:
                技巧：可以通过保持frame为none而修改default为其它不存在的设备号来自动触发对ros默认图像话题的连接。
            mode默认为hsv，另外还可以使用YUV（YCrCb）、LAB。
        """
        # 鲁棒的连接处理
        flag=0
        if frame_dev_topic is None: frame_dev_topic = default;flag = 1
        # frame_dev_topic本身就是图片
        if isinstance(frame_dev_topic,ndarray):
            def get_img(): return frame_dev_topic
        # 尝试device
        elif isinstance(frame_dev_topic,int):
            try:
                if cls.cap_dict.get(frame_dev_topic) is None:  # 还没有cap过
                    cls.Cap(frame_dev_topic)  # 重新还行cap
            except:
                if flag == 1:  # 表明报错是因为default的值有问题
                    frame_dev_topic = "camera/color/image_raw"
                    print('连接错误：未找到视觉设备，尝试连接ROS')  # 尝试连接ROS
                else: cls.Cap(frame_dev_topic)  # 重新尝试连接
            else:
                print(f'成功连接设备号：{frame_dev_topic}')
                def get_img():
                    return cls.Read(frame_dev_topic)
        # 尝试topic
        if isinstance(frame_dev_topic,str):
            import subprocess
            try:
                p = subprocess.getoutput("pgrep rosmaster")
                if p == '': exit('连接错误：rosmaster未开启，使用ROS模式请先启动roscore')
                NODE_NAME = 'color_thresh_determine'
                if rospy.get_name() == '':
                    rospy.init_node(NODE_NAME)
                    rospy.loginfo("Initializing {} node.".format(NODE_NAME))
                rospy.wait_for_message("camera/color/image_raw",Image,timeout=1)
            except:
                if flag: exit('连接错误：为查询到图片消息，获取消息失败\r\n两种方式均失败，请明确采用的方式及参数后重试')
                else: exit(f'给定话题{frame_dev_topic}无法获取图片消息')
            else:
                print(f'成功连接ROS话题:{frame_dev_topic}')
                def get_img():
                    img = rospy.wait_for_message(frame_dev_topic,Image)
                    return CvBridge().imgmsg_to_cv2(img,"bgr8")

        # 创建窗口和滑条
        def callback(*arg): pass  # 创建空回调函数（比起直接置0更易拓展）
        WINDOW_NAME = 'Color pick(按ESC键退出)'
        cv2.namedWindow(WINDOW_NAME,cv2.WINDOW_NORMAL)
        if mode in ['HSV','hsv']:
            cv2.createTrackbar('LH', WINDOW_NAME, 0, 179, callback)
            cv2.createTrackbar('LS', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('LV', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('HH', WINDOW_NAME, 0, 179, callback)
            cv2.createTrackbar('HS', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('HV', WINDOW_NAME, 0, 255, callback)
        elif mode in ['YUV','YCrCb']:
            cv2.createTrackbar('LY ', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('LCr', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('LCb', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('HY ', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('HCr', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('HCb', WINDOW_NAME, 0, 255, callback)
        elif mode in ['LAB','Lab','lab']:
            cv2.createTrackbar('LL', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('LA', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('LB', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('HL', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('HA', WINDOW_NAME, 0, 255, callback)
            cv2.createTrackbar('HB', WINDOW_NAME, 0, 255, callback)
        # 主循环
        while True:
            if mode in ['HSV','hsv']:
                LH = cv2.getTrackbarPos('LH', WINDOW_NAME)
                LS = cv2.getTrackbarPos('LS', WINDOW_NAME)
                LV = cv2.getTrackbarPos('LV', WINDOW_NAME)
                HH = cv2.getTrackbarPos('HH', WINDOW_NAME)
                HS = cv2.getTrackbarPos('HS', WINDOW_NAME)
                HV = cv2.getTrackbarPos('HV', WINDOW_NAME)
                ColorL = [LH, LS, LV]
                ColorH = [HH, HS, HV]
                img = get_img()
                res, mask = StdVideo.color_filter(img,ColorL,ColorH,convert=cls.Types.COLOR_BGR2HSV)
            elif mode in ['YUV','YCrCb']:
                LY = cv2.getTrackbarPos('LY ', WINDOW_NAME)
                LCr = cv2.getTrackbarPos('LCr', WINDOW_NAME)
                LCb = cv2.getTrackbarPos('LCb', WINDOW_NAME)
                HY = cv2.getTrackbarPos('HY ', WINDOW_NAME)
                HCr = cv2.getTrackbarPos('HCr', WINDOW_NAME)
                HCb = cv2.getTrackbarPos('HCb', WINDOW_NAME)
                ColorL = [LY, LCr, LCb]
                ColorH = [HY, HCr, HCb]
                img = get_img()
                res, mask = StdVideo.color_filter(img,ColorL,ColorH,convert=cls.Types.COLOR_BGR2YCrCb)
            elif mode in ['Lab','LAB']:
                LL = cv2.getTrackbarPos('LL', WINDOW_NAME)
                LA = cv2.getTrackbarPos('LA', WINDOW_NAME)
                LB = cv2.getTrackbarPos('LB', WINDOW_NAME)
                HL = cv2.getTrackbarPos('HL', WINDOW_NAME)
                HA = cv2.getTrackbarPos('HA', WINDOW_NAME)
                HB = cv2.getTrackbarPos('HB', WINDOW_NAME)
                ColorL = [LL, LA, LB]
                ColorH = [HL, HA, HB]
                img = get_img()
                res, mask = StdVideo.color_filter(img,ColorL,ColorH,convert=cls.Types.COLOR_BGR2LAB)
            else: exit('错误：请输入正确的mode参数值')
            # 显示
            mask = np.dstack((mask,mask,mask))  # 为了实现并排显示，需要将二值图变成3d图
            try:
                StdVideo.Show(WINDOW_NAME,(mask,res),window_size=(1280,720),window_mode=None)
                if show_raw: StdVideo.Show('raw_img(按ESC键退出)',img,window_size=(640,480))
            except Exception as e:
                print(e)
                print(f"最终确定的阈值为：{mode}_L={ColorL},{mode}_H={ColorH}")
                if save_path is not None:
                    cls.save_parameters(ColorL+ColorH,save_path)
                break
        cv2.destroyWindow(WINDOW_NAME)
        return ColorL,ColorH  # 返回确定阈值

    @classmethod
    def color_replace(cls,frame:ndarray,color:Union[int,tuple,list],area:Union[tuple,list]):
        """
            将图像中的某些区域替换为
                area = ([])  # TODO
        """
        if frame.ndim == 2:
            pass
        else:
            # # 裁切图像，避免由于物块大小不均匀造成的下层物块在上层轮廓侧边的漏出误识别
            # bi_cp = StdVideo.create_empty_frame(ImageSize[:2],0)
            # bi_cp[:,281:359] = binary_img[:,281:359]
            # binary_img = bi_cp
            if not isinstance(color,int):
                color = color[0]
            cls.create_empty_frame()
            pass

    @classmethod
    def gradient(cls,frame,mode:str,ksize=5,convert=cv2.COLOR_BGR2GRAY):
        """
            mode:
                sobelx,sobely,sobel
                scharrx,scharry,scharr
                laplace
            ksize: scharr系列始终为3，其它默认为5
        """
        if convert is not None: frame = cv2.cvtColor(frame,convert)
        if mode in ['sobelx','scharrx']:
            if mode == 'scharr': ksize = -1
            g_frame = cv2.Sobel(frame,cv2.CV_64F,1,0,ksize=ksize)
        elif mode in ['sobely','scharry']:
            if mode == 'scharr': ksize = -1
            g_frame = cv2.Sobel(frame,cv2.CV_64F,0,1,ksize=ksize)
        elif mode == 'sobelxy':
            g_frame = cv2.Sobel(frame,cv2.CV_64F,1,1,ksize=ksize)
        elif mode == 'sobel':
            g_frame = cls.Math.plus([cv2.Sobel(frame,cv2.CV_64F,0,1),cv2.Sobel(frame,cv2.CV_64F,1,0)])
        elif mode == 'scharr':
            g_frame = cls.Math.plus([cv2.Scharr(frame,cv2.CV_64F,0,1),cv2.Scharr(frame,cv2.CV_64F,1,0)])
        elif mode == 'laplace':
            g_frame = cv2.Laplacian(frame,cv2.CV_64F)
        else: raise Exception('梯度mode错误')
        return np.uint8(np.absolute(g_frame))

    @classmethod
    def canny_determine(cls):
        """ 通过滑动条动态更改canny函数的参数 """
        # 创建空回调函数（比起直接置0更易拓展）
        def callback(*arg):
            pass

        if not hasattr(cls.canny_determine,'first'):
            cls.canny_determine.__dict__['first'] = False
            cv2.createTrackbar('min_val',    'EdgesDetect', 0, 255, callback)
            cv2.createTrackbar('max_val',    'EdgesDetect', 0, 254, callback)
            cv2.createTrackbar('sobel_size', 'EdgesDetect', 3, 7, callback)
            cv2.createTrackbar('precise',    'EdgesDetect', 0, 1, callback)        
        min_val = cv2.getTrackbarPos('min_val', 'EdgesDetect')
        max_val = cv2.getTrackbarPos('max_val', 'EdgesDetect')
        sobel_size = cv2.getTrackbarPos('sobel_size', 'EdgesDetect')
        precise = cv2.getTrackbarPos('precise', 'EdgesDetect')

        if sobel_size<5:
            sobel_size = 3
        if 5<=sobel_size<7:
            sobel_size = 5
        else:
            sobel_size = 7
        if precise>=1:
            precise = True
        else:
            precise = False

        return min_val,max_val,sobel_size,precise

    @staticmethod
    def edge_detect(frame,min_val,max_val,sobel_size,precise,convert=cv2.COLOR_BGR2GRAY):  # 第三个参数越大边缘越多，第四个参数为True能有效减少杂散边缘
        """
            采用经典最佳的canny算法:
                强度梯度大于 maxVal 的任何边缘必定是边缘，而小于 minVal 的那些边缘必定是非边缘，因此
                将其丢弃。介于这两个阈值之间的对象根据其连通性被分类为边缘或非边缘。如果将它们连接
                到“边缘”像素，则将它们视为边缘的一部分。否则，它们也将被丢弃。
        """
        # 将新的颜色图进行灰度化
        if convert is not None: frame = cv2.cvtColor(frame,convert)
        # 对灰度化图进行canny边缘检测（canny自带了5x5高斯滤波器消除图像中的噪声）
        edges_img = cv2.Canny(frame,min_val,max_val,None,sobel_size,precise)
        return edges_img

    @staticmethod
    def create_empty_frame(size:Union[tuple,list],color:Union[tuple,list,int]):
        """
            创建纯色空图：
                size可以为2元或3元
                color可以为灰度值或bgr值
                上述两者应对应好
        """
        if isinstance(color,int): channels = 1  # 利用np的维度判断函数对数据的维度进行判断
        else:
            channels = len(color)  # 根据颜色数量
            if channels == 2: exit('颜色维度不能为2，只能为1（灰度or二值图）和3（彩色图）')
            if channels == 1: color = color[0]

        if channels == 3:
            src = np.zeros([size[1],size[0],3], dtype=np.uint8)  # dtype在调试时非常重要，因为OpenCV-Python代码中的大量错误是由无效的数据类型引起的
            for i in range(3): src[:,:,i] += color[i]
        else:
            src = np.zeros([size[1],size[0]], dtype=np.uint8) + color

        return src

    @staticmethod
    def save_parameters(params,path:str):
        """ 保存参数 """
        import json
        FILE_NAME = path
        with open(FILE_NAME, 'w') as f_obj:
            json.dump(params ,f_obj)
        print('参数已存储到指定文件中')

    @staticmethod
    def load_parameters(path:str):
        """ 载入保存的参数 """
        import json
        FILE_NAME = path
        with open(FILE_NAME) as f_obj:
            return json.load(f_obj)

    class Color(object):
        pass

    class Threshold(object):
        """ 用于图像二值化 """
        @staticmethod
        def color_range(frame:ndarray, ColorL:list, ColorH:list,convert=cv2.COLOR_BGR2HSV):
            """ 根据颜色空间范围选择合适的颜色阈值对彩色图像（默认为BGR格式）进行处理 """
            frame_cp = frame.copy()
            cvt = cv2.cvtColor(frame_cp,convert)
            lower_thresh = np.array(ColorL)
            upper_thresh = np.array(ColorH)
            mask = cv2.inRange(cvt, lower_thresh, upper_thresh)
            res = cv2.bitwise_and(frame_cp, frame_cp, mask=mask)
            return res, mask
        @staticmethod
        def gray_global(frame,threshold_value,threshold_max,mode=cv2.THRESH_BINARY,convert_color=True,return_thresh=False):
            """ 灰度图像全局二值化 """
            if convert_color: frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            thresh, new_img = cv2.threshold(frame,threshold_value,threshold_max,mode)
            if return_thresh: return thresh,new_img
            else: return new_img
        @staticmethod
        def gray_adaptive(frame,maxValue,method=cv2.ADAPTIVE_THRESH_GAUSSIAN_C,thresholdType=cv2.THRESH_BINARY,blockSize=3,C=0,convert_color=True):
            """
                src：需要进行二值化的一张灰度图像
                maxValue：满足条件的像素点需要设置的灰度值（将要设置的灰度值）
                method：自适应阈值算法。可选ADAPTIVE_THRESH_MEAN_C 或 ADAPTIVE_THRESH_GAUSSIAN_C
                thresholdType：opencv提供的二值化方法，只能THRESH_BINARY或者THRESH_BINARY_INV
                blockSize：要分成的区域大小，上面的N值，一般取奇数
                C：常数，每个区域计算出的阈值的基础上在减去这个常数作为这个区域的最终阈值，可以为负数
            """
            if convert_color: frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            return cv2.adaptiveThreshold(frame,maxValue,method,thresholdType,blockSize,C)
        @staticmethod
        def gray_otsu(frame,threshold_value,threshold_max,mode=cv2.THRESH_BINARY,convert_color=True,return_thresh=False):
            """ 大津法自适应阈值（参数与灰度全局二值化相同） """
            if convert_color: frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            thresh, new_img = cv2.threshold(frame,threshold_value,threshold_max,mode+cv2.THRESH_OTSU)
            if return_thresh: return thresh,new_img
            else: return new_img

    class Geometry(object):
        """
            OpenCV提供了两个转换函数**cv.warpAffine**和**cv.warpPerspective**，使用它们进行各种转换。
                **cv.warpAffine**采用2x3转换矩阵，而**cv.warpPerspective**采用3x3转换矩阵。
            该类目前仅实现了常用的放缩和旋转操作，而其它诸如平移、仿射、透视变换则没有。
        """
        @staticmethod
        def Resize(frame:ndarray,new_shape:Optional[tuple],fx=None,fy=None,precise=False):
            """
                缩小首选的插值方法是：cv.INTER_AREA;放大的方法有：cv.INTER_CUBIC（慢）和cv.INTER_LINEAR。
                为此，该函数进行了一些判断，以选择较好的方式，并提供precise参数来选择在放大时是否使用较慢但更细致的方法。
                将new_shape设为none后，fx和fy将生效，可以按倍数调整。
            """
            if new_shape is None:
                new_shape = (frame.shape[0]*fx,frame.shape[1]*fy)
            if new_shape[0]>frame.shape[0] or new_shape[1]>frame.shape[1]:
                if precise: interpolation = cv2.INTER_CUBIC
                else: interpolation = cv2.INTER_LINEAR
            else: interpolation = cv2.INTER_AREA

            res = cv2.resize(frame.copy(),new_shape,interpolation=interpolation)
            return res

        @staticmethod
        def Rotation(frame:ndarray,angle,center=None,scale=1):
            """ center为none默认图像中心为旋转中心（旋转是逆时针的）；scale为旋转后图像的放缩倍数 """
            rows_cols = frame.shape
            if center is None: center = ((rows_cols[1]-1)/2.0,(rows_cols[0]-1)/2.0)
            M = cv2.getRotationMatrix2D(center,angle,scale)  # 得到变换矩阵：cols-1 和 rows-1 是坐标限制
            dst = cv2.warpAffine(frame,M,(rows_cols[1],rows_cols[0]))
            return dst

    class Math(object):
        """ 图像的各种算数、位运算 """ 
        @staticmethod
        def plus(imgs:List[Union[ndarray,float]],weights:Union[None,list,tuple]=None):
            """ 饱和相加，imgs最后元素允许为标量，标量不给予权重 """
            if isinstance(imgs[-1],(int,float)): scalar = imgs.pop(-1)
            else: scalar = 0
            if weights is None:
                weights = [1 for _ in range(len(imgs))]
            img_p = cv2.addWeighted(imgs.pop(0),weights[0],imgs.pop(0),weights[1],scalar)
            for img in imgs:
                img_p = cv2.addWeighted(img_p,1,img,weights[2],0)
            return img_p

        @classmethod
        def bitwise(*args):
            """
                这包括按位AND OR XOR NOT操作。它们在提取图像的任何部分、定义和处理非矩形ROI等方面非常有用。
                参数按输入示例(执行顺序始终从左至右)：
                    带not：bitwise('not',img1,'and','not',img2,'or',img3,'xor',img4)
                    无not：bitwise(img1,'and',img2,'or',img3,'xor',img4)
                该函数虽然方便，但是却较大地影响了处理速度，择需使用。
            """
            def __bitwise_X(a,operator,b=None):
                if operator == 'and':
                    a = cv2.bitwise_and(a,b)  # 注意区别cv2.bitwise_and(a,a,mask=b)的写法，在提取特定区域时，不直接写成cv2.bitwise_and(a,mask)，其意义？
                elif operator == 'or':
                    a = cv2.bitwise_or(a,b)
                elif operator == 'xor':
                    a = cv2.bitwise_xor(a,b)
                elif operator == 'not':
                    a = cv2.bitwise_not(a)
                else: exit('算符错误')
                return a
            args = list(args)
            # 先对需要求反的图像进行求反
            while 'not' in args:
                next = args.index('not')+1
                args[next] = cv2.bitwise_not(args[next])
                args.pop(next-1)
            # 然后执行AND OR XOR操作
            i = 0
            len_ = len(args)-1
            while i < len_:
                args[i+2] = __bitwise_X(args[i],args[i+1],args[i+2])
                i+=2
            return args[-1]  # 最后一个元素保存着最后位运算完的结果

    class Morphology(object):
        """ 图像形态学：腐蚀、膨胀、开运算、闭运算 """
        def kernel(size,shape=cv2.MORPH_RECT):
            """ 获得各种形状的核：cv2.MORPH_RECT（矩形）、cv2.MORPH_CROSS（十字形）、cv2.MORPH_ELLIPSE（(椭)圆形） """
            return cv2.getStructuringElement(shape,size)
        def erode(frame:ndarray,kernel=(3,3),iterations=1):
            """ 腐蚀（侵蚀） """
            if isinstance(kernel,tuple): kernel = np.ones(kernel,np.uint8)
            return cv2.erode(frame,kernel,iterations=iterations)
        def dilate(frame:ndarray,kernel=(3,3),iterations=1):
            """ 膨胀（扩张） """
            if isinstance(kernel,tuple): kernel = np.ones(kernel,np.uint8)
            return cv2.dilate(frame,kernel,iterations=iterations)
        def open(frame,kernel=(3,3)):
            """
            一个白色封闭圆环，open先腐蚀后膨胀，从而，这个圆环中薄弱的地方被切断，实现了“open”
            它对于消除噪音很有用
            """
            if isinstance(kernel,tuple): kernel = np.ones(kernel,np.uint8)
            return cv2.morphologyEx(frame,cv2.MORPH_OPEN, kernel)
        def close(frame,kernel=(3,3)):
            """
            一个白色断裂圆环，close先膨胀后腐蚀，从而，这个圆环中切断的地方被连接，实现了“close”
            在关闭前景对象内部的小孔或对象上的小黑点时很有用
            """
            if isinstance(kernel,tuple): kernel = np.ones(kernel,np.uint8)
            return cv2.morphologyEx(frame,cv2.MORPH_CLOSE, kernel)
        def gradient(frame,kernel=(3,3)):
            """
            结果看起来像是图像的轮廓
            """
            if isinstance(kernel,tuple): kernel = np.ones(kernel,np.uint8)
            return cv2.morphologyEx(frame,cv2.MORPH_GRADIENT, kernel)
        def top_hat(frame,kernel=(3,3)):
            """
            输入图像和图像开运算之差。可以得到只有图像中的噪声的图像。仿佛是将噪声的帽子顶了出来。
            """
            if isinstance(kernel,tuple): kernel = np.ones(kernel,np.uint8)
            return cv2.morphologyEx(frame,cv2.MORPH_TOPHAT, kernel)
        def black_hat(frame,kernel=(3,3)):
            """
            输入图像和图像闭运算之差。可以得到只有图像中的空洞（此时变成白色）的图像。仿佛是给黑洞带个帽子。
            """
            if isinstance(kernel,tuple): kernel = np.ones(kernel,np.uint8)
            return cv2.morphologyEx(frame,cv2.MORPH_BLACKHAT, kernel)

    class Filter(object):
        """
        与一维信号一样，还可以使用各种低通滤波器（LPF），高通滤波器（HPF）等对图像进行滤波。
        LPF有助于消除噪声，使图像模糊等。HPF滤波器有助于在图像中找到边缘。
        """
        pass  # TODO

    class Contour(object):
        """轮廓查找与检测"""
        contour_info = {}  # 维护轮廓的有关信息：ID-信息（ID相同的轮廓信息会相互覆盖）

        @staticmethod
        def find(binary_img,mode=cv2.RETR_EXTERNAL,approx=cv2.CHAIN_APPROX_NONE,return_hierarchy=False):
            """
            contours：list结构，列表中每个元素代表一个边沿信息。每个元素是(x,1,2)的三维向量，x表示该条边沿里共有多少个像素点，第三维的那个“2”表示每个点的横、纵坐标；
                如果输入选择cv2.CHAIN_APPROX_SIMPLE，则contours中一个list元素所包含的x点之间应该用直线连接起来，这个可以用cv2.drawContours()函数观察一下效果。
            hierarchy：(x,4)的二维ndarray。x和contours里的x是一样的意思。
                如果输入选择cv2.RETR_TREE，则以树形结构组织输出，hierarchy的四列分别对应下一个轮廓编号、上一个轮廓编号、父轮廓编号、子轮廓编号，该值为负数表示没有对应项。
            """
            contours,hierarchy = cv2.findContours(binary_img,mode,approx)
            if return_hierarchy:
                return contours,hierarchy
            else:
                return contours

        @staticmethod
        def area(contour):
            """ 获得轮廓的面积 """
            return cv2.contourArea(contour)
        @staticmethod
        def arc_lenth(contour,close=True):
            """ 当轮应该是闭合的时候，close设为true """
            return cv2.arcLength(contour,close)
        @classmethod
        def rect(cls,contour,mode=0,box_point=True):
            """ 获得轮廓的外接矩形 """
            if mode == 0:
                rect = cv2.minAreaRect(contour)
                (cx,cy), (width, height), rotation_angle = rect
            elif mode == 1: return
            if box_point:
                return rect,cls.box_point(rect)
            else:
                return (cx,cy), (width, height), rotation_angle
        @staticmethod
        def box_point(rect):
            """ 获取矩形的四个顶点坐标 """
            box = cv2.boxPoints(rect)
            box_int = np.int0(box)  # 浮点转整形，用于draw
            return box,box_int
        @classmethod
        def approximate(cls,contour,eps,cnt_lenth=None,close=True):
            """
                获得轮廓的近似:
                    eps表示以轮廓周长cnt_lenth为参考的近似程度，越大相似程度越低
                    cnt_lenth为None时将根据传入的轮廓自动计算
                    close参数指定曲线是否是闭合的
                该函数可用于在拟合外接矩形前对轮廓进行一些处理，特别是去掉一些
            """
            if cnt_lenth is None: cnt_lenth = cls.arc_lenth(contour)
            epsilon = eps*cnt_lenth
            return cv2.approxPolyDP(contour,epsilon,close)
        @staticmethod
        def isContourConvex(approx):
            """ 判断轮廓是否凸包 """ 
            return cv2.isContourConvex(approx)
        @staticmethod
        def draw(frame,cnts_box:list,color=(0,0,255),thickness=2,index=-1):
            """
                cnt_box可以是轮廓外接矩形的四个顶点，也可以是轮廓
                单个轮廓也应该是list类型，即[cnt]
            """
            cv2.drawContours(frame,cnts_box,index,color,thickness)

        @classmethod
        def NFC_F(cls,frame_cnts,area_limit:tuple,ratio_max:float,mid_xy:tuple,mid_z:float=None):
            """
                Nearest fit contour finder
                给定面积的范围、最大的长宽比例（指轮廓的最小外接矩形）、参考中位值（用于确定最近的轮廓），从所有轮廓中选择最合适的那个。
                frame_cnts可以是图像或轮廓，若是前者，则自动使用默认参数完成图像的轮廓查找。
            """
            nearest_dis = mid_xy[0] + mid_xy[1]
            if mid_z is not None: mid_z_sqrt = math.sqrt(mid_z)
            nearest_cnt = None
            if isinstance(frame_cnts,ndarray): frame_cnts = cls.find(frame_cnts)
            if area_limit[0] + area_limit[1] < 2:  # 百分比面积
                area = ImageSize[0]*ImageSize[1]
                area_limit[0] = area*area_limit[0]
                area_limit[1] = area*area_limit[1]
            for cnt in frame_cnts:  # 首先是轮廓面积初筛，然后是拟合矩形的面积再次筛选，然后是长宽比筛选
                cnt_Area = cls.area(cnt)  # 计算面积
                if (area_limit[0] < cnt_Area < area_limit[1]):  # 实际的物块距离是可知的，相机又是确定的，因此，物块的面积必然在一个较小的范围内
                    # print(cnt_Area,'\r\n')  # 调试打印面积信息
                    # 求最小外接矩形并获取其参数
                    r_rect = cls.rect(cnt,mode=0,box_point=False)  # 获取最小外接矩形(它返回一个Box2D结构，其中包含以下细节 -(中心(x,y)，(宽度，高度)，旋转角度))
                    (cx,cy), (width, height), rotation_angle = r_rect  # https://blog.csdn.net/weixin_43229348/article/details/125986969
                    rect_area = width*height
                    # 判断矩形面积是否合理，同时边长是否符合长宽比
                    if (area_limit[0] < rect_area < area_limit[1]) and (1/ratio_max < width/height < ratio_max):
                        x_bias = mid_xy[0]-cx  # 偏差为正时，即cx在偏左的地方，机械臂y轴左移
                        y_bias = mid_xy[1]-cy  # 偏差为正时，即cy在偏上的地方，机械臂x轴前移
                        if mid_z is not None:
                            z_bias = math.sqrt(rect_area) - mid_z_sqrt
                        else: z_bias = 0
                        new_dis = abs(x_bias) + abs(y_bias) + abs(z_bias)  # 与欧氏距离本质相同
                        # 最终筛选出最接近中心的那个方块
                        if new_dis < nearest_dis:
                            nearest_dis = new_dis
                            # 顺带记录最近的方块的信息
                            nearest_cnt = cnt
                            nearest_rect = r_rect
                            nearest_center = (cx,cy)
                            nearest_w_h = [width, height]
                            nearest_rotation = -rotation_angle  # 添加负号保持一致
                            if mid_z is not None:
                                nearest_bias = [x_bias,y_bias,z_bias]
                            else: nearest_bias = [x_bias,y_bias,0]

            # 找到最近了轮廓后获取该轮廓的详细信息
            if nearest_cnt is not None:
                nearest_w_h = sorted(nearest_w_h)  # 始终是w<h
                _,nearest_box_int = cls.box_point(nearest_rect)  # 使用cv2.boxPoints()可获取该矩形的四个顶点坐标
                # print("area:{}".format(cnt_Area))  # 打印最近轮廓的面积
                return nearest_center,nearest_bias,nearest_rotation,nearest_box_int,nearest_w_h,nearest_cnt,
            return None,None,None,None,None,None

        # 获得轮廓信息
        @classmethod
        def get_info(cls,contour,infomode):
            """
                contour可以是id（int型,前提是保证已经存储了有关信息，若没有则将报错）或轮廓本身。
                infomode可以选择为：less:;'base':;'norm':;more;most;
            """
            if isinstance(contour,int):pass

    class Draw(object):
        """
            绘制各种图像：几种基本图形内容、准星、鼠标绘图等
            注意：默认所绘制内容直接覆盖原图(带有cover参数的可以通过设置为false不破坏原图，没有的请自行先copy)
        """
        @staticmethod
        def basic_draw(frame,pos1_c,pos2_r=(None,),color=(0,0,0),thickness=1,line_type=cv2.LINE_8,elipse_angles=(0,0,0),font=cv2.FONT_HERSHEY_PLAIN,shape_convert=False,close=True,cover=True,tips=''):
            """
                根据传入的参数特点，自动确定绘制何种内容（直线和矩形的区分部分是靠shape_convert为false或true）
                支持绘制的内容有：圆和椭圆、文本、多段直线和多边形、单段直线和矩形
            """
            if not cover: frame = frame.copy()
            if not isinstance(pos2_r,tuple) or shape_convert=='circle':  # 例：cv2.circle(src, (250, 250), 150, (0, 0, 255), 4, cv2.LINE_8, 0)  # 圆
                cv2.circle(frame,pos1_c,pos2_r,color,thickness,line_type,0)  # 圆
            elif elipse_angles!=(0,0,0) or shape_convert=='elipse':  # 例：cv2.ellipse(src, (250, 250), (150, 50), 360, 0, 360, (255, 234, 0), 3, cv2.LINE_8, 0)  # 椭圆
                cv2.ellipse(frame,pos1_c,pos2_r,*elipse_angles,color,thickness,line_type,0)  # 椭圆
            elif isinstance(pos1_c,str):
                cv2.putText(frame,pos1_c,pos2_r,font,thickness,color, 2, cv2.LINE_8)  # 文字
            elif len(pos1_c[0])>1:  # 绘制(多段)直线
                len_ = len(pos1_c)
                if len_ ==2:  # 绘制一条直线
                    cv2.line(frame,pos1_c[0],pos1_c[1],color,thickness,line_type,0)  # 直线
                else:
                    for index,point in enumerate(pos1_c):
                        cv2.line(frame, point, pos1_c[(index + 1) % len_],color,thickness,line_type,0)
                        if not close and index==len_ - 2: break  # 不封闭直接退出
            elif shape_convert:  # 例：cv2.line(src, (10, 10), (400, 400), (255, 0, 0), 1, cv2.LINE_8, 0)
                cv2.line(frame,pos1_c,pos2_r,color,thickness,line_type,0)  # 直线
            else: cv2.rectangle(frame,pos1_c,pos2_r,color,thickness,line_type,0)  # 正方形
            if not cover: return frame

        @staticmethod
        def front_sight(frame,x,y,corlors=((0,0,255),(0,0,255)),corver=True):
            """ 绘制瞄准准星 """
            x,y = int(x),int(y)
            start,end = 10,40
            if not corver: frame = frame.copy()
            frame = cv2.circle(frame, (x, y), 17,corlors[0], 2)
            frame = cv2.line(frame, (x, y+start), (x, y+end), corlors[1], 2, cv2.LINE_8, 0)
            frame = cv2.line(frame, (x+start, y), (x+end, y), corlors[1], 2, cv2.LINE_8, 0)
            frame = cv2.line(frame, (x, y-start), (x, y-end), corlors[1], 2, cv2.LINE_8, 0)
            frame = cv2.line(frame, (x-start, y), (x-end, y), corlors[1], 2, cv2.LINE_8, 0)
            if not corver: return frame

        @staticmethod
        def border():  # TODO
            """ 为图像添加边框 """
            pass

        @classmethod
        def mouse_draw(cls,window_name=None,frame=None,color=(0,0,0),thickness=2):
            """
                鼠标在给定的窗口上绘图。显然window_name和frame之间具有一定的关联性，但是frame是每帧不断变化的，因此这种关联是不固定的。
                为此需要每次进行更新，这是通过bind_to的绑定属性来实现联系的。
                实际上，对一幅图进行描绘的时候图片应该是固定的，或者说变化的频率是不快的，否则上一幅图片还没绘制好就刷新了要重画。
                只不过，考虑到有时候我们需要在对图片进行标记的时候需要一幅一幅地连续处理，因此这样的操作还是有用的。
            """
            def bind_to(window_name,frame):
                """ 所有的绑定函数都有相同的参数：第一个是窗口名，第二个是frame """
                if not hasattr(bind_to,window_name):  # 同一个窗口仅执行一次回调函数设置
                    drawing = False
                    bind_to.__dict__[window_name] = frame
                    if window_name not in StdVideo.windows_set:  # 为保证setMouseCallback不出错，先创建一个空窗口
                        cv2.namedWindow(window_name)
                    def call_back(event,x,y,flags,param):
                        nonlocal drawing
                        if event == cv2.EVENT_LBUTTONDOWN:
                            drawing = True
                            cv2.circle(bind_to.__dict__[window_name],(x,y),thickness,color,-1)
                        elif event == cv2.EVENT_MOUSEMOVE:
                            if drawing == True:
                                cv2.circle(bind_to.__dict__[window_name],(x,y),thickness,color,-1)
                        elif event == cv2.EVENT_LBUTTONUP:
                            drawing = False
                    cv2.setMouseCallback(window_name,call_back)
                else: bind_to.__dict__[window_name] = frame
            # 立刻执行绑定
            if None not in (window_name,frame): bind_to(window_name,frame)
            # 返回绑定函数
            else: return bind_to

        @staticmethod
        def draw_contour(frame,cnt_box,color=(0,0,255),thickness=2):
            return StdVideo.Contour.draw(frame,cnt_box,color,thickness)

    class Histogram(object):
        """ 直方图相关处理 """  # TODO 
        def calculate(dim=1):
            """ 查找直方图，dim选择是1维or2维 """
            pass

        def plot():
            """ 绘制直方图 """
            pass

        def equalize():
            """ 直方图均衡化 """
            pass

        def back_project():
            """ 直方图反投影 """
            pass

    class UnitConvert(object):
        """ 该类主要实现像素单位和米之间的对应转换 """
        camrera_info_dict = {}  # 维护设备号（或ROS话题名）-相机信息（对单位转换有用的信息）
        default_topic_name = "/camera/color/camera_info"
        unit_coefficient = {}  # 维护设备号（或ROS话题名）-m_per_pixel转换关系

        @classmethod
        def get_camera_info(cls,path='',topic=''):
            """
                自动获取相机有关信息，其中path为用相机排到的某幅图片的位置。
                注意图片必须是相机原生拍摄或使用系统相机软件拍摄得到的，而
                不能是用OpenCV读取后保存的，因为这种情况下图片其它信息丢失了。
            """
            if topic != '':  # 从ROS话题获取消息
                if topic == '': topic = cls.default_topic_name
                info = rospy.wait_for_message(topic,CameraInfo)
                cls.camrera_info_dict[topic] = [info.K[0],info.K[4]]
                print("成功获取相机信息:fx={},fy={}.".format(info.K[0],info.K[4]))
            elif path != '':
                with open('/home/ghz/img.jpg','rb') as f:
                    import exifread
                    tags = exifread.process_file(f)
                try:
                    FL = tags.get('EXIF FocalLength').values[0]
                    DZR = tags.get('EXIF DigitalZoomRatio')
                    if DZR is None: DZR = 1
                    else: DZR = DZR.values[0]
                    cls.camrera_info_dict[topic] = [FL,DZR]
                except: raise Exception('错误：无法提取指定相机的有关信息，您的相机可能不具备相关信息的输出，请查阅手册自行获取')

            return cls.camrera_info_dict[topic]

        @classmethod
        def set_camera_info(cls,device,f_sensor:Optional[list]=None,fx=None,fy=None):
            """
            若无法自动获取相机信息，请自行查阅资料并获取如下有关信息，通过该函数进行设置：
                相机的镜头焦距+感光芯片尺寸，或者，fx或fy。
                两种方式二选一，如果都选，则默认使用第一种。
            """
            if f_sensor is not None:
                cls.camrera_info_dict[device] = f_sensor
            else:
                cls.camrera_info_dict[device] = [fx,fy]

        @classmethod
        def calculate_m_pixel_coefficient(cls,device,x_or_y='x')->float:
            """ 得到在distance为1m时的像素-米对应关系，该关系与distance成正比 """
            if isinstance(device,str):
                if device == '': device = cls.default_topic_name
                if x_or_y=='x':
                    fx = cls.camrera_info_dict[device].K[0]  # k为相机内参矩阵，python中实际表示为一个tuple：(fx,0,cx,0,fy,cy,0,0,1)
                    m_per_pixel_dis1m = 1/fx
                else:
                    fy = cls.camrera_info_dict[device].K[4]  # 一般以fx为准
                    m_per_pixel_dis1m = 1/fy
            else:
                pixels_length = 1
                distance = 1
                f,dzr = cls.camrera_info_dict[device]
                m_per_pixel_dis1m = pixels_length * distance / f / 5.62 * 0.8 / 1000 / dzr * 0.925
            cls.unit_coefficient[device] = m_per_pixel_dis1m
            return m_per_pixel_dis1m

        @classmethod
        def convert_pixel_bias_to_meter(cls,device,pixel_bias,distance):
            """ 转换像素数-meter """
            return cls.unit_coefficient[device]*distance*pixel_bias

        @classmethod
        def convert_meter_bias_to_pixels(cls,device,meter_bias,distance):
            """ 转换meter-像素数 """
            return 1/(cls.unit_coefficient[device]*distance) * meter_bias

            # 仅spin一次(默认sleep有0.5秒sleep时间)，请在函数外面套一个while循环

    @classmethod
    def ros_spin_once(cls,sleep_time=0.5):
        """ 该函数改版自rospy.spin，实现了python中ros的单次spin的spinonce函数 """
        if not hasattr(cls.ros_spin_once,'first'):
            cls.ros_spin_once.__dict__['first'] = False
            if not rospy.core.is_initialized():
                raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
            rospy.core.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
        try:
            if not rospy.core.is_shutdown(): rospy.rostime.wallsleep(sleep_time)
            else: exit('\r\nROS Core Shut Down')
        except KeyboardInterrupt:
            rospy.core.logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')
