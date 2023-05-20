
import time
import rospy
from std_msgs.msg import Float32
import threading
from threading import Event
import numpy as np
import cv2
from Best_Contour import StdVideo
from moveit_commander import MoveGroupCommander,RobotCommander,JointState,RobotState
import tf_conversions
import termios,tty,select,sys,os  # 获取键盘值要额外用到的标准库
""" CPU:i7-12th """

class wait_for_message_test(object):
    """ 实例化该类以测试wait_for_message的性能。可以看到其延迟在7ms以上。因此尽量不要用。 """
    def __init__(self) -> None:
        rospy.init_node('test')
        self.trance = Float32()
        self.pose_pub = rospy.Publisher("/ar",Float32,queue_size=10)
        self.trance.data = 0
        self.sub = rospy.Subscriber("/ar",Float32,self.except_producer,queue_size=1)
        threading.Thread(target = self.pub,daemon=True).start()
        self.start = 0
        start = time.time()
        info = rospy.wait_for_message("/ar",Float32)
        end = time.time()
        print('一次性获取消息时间为',end-start,'内容为：',info)
        rospy.spin()
    def except_producer(self,message:Float32):
        self.message = message
        end = time.time()
        if not self.start == 0: print(end-self.start)
        rospy.sleep(1)
        self.start = time.time()

    def pub(self):
        times = 0
        self.pub.__dict__['thread'] = True
        while True:
            times+=1
            self.trance.data += 1
            self.pose_pub.publish(self.trance)
            rospy.sleep(0.1)
            if times == 6:break

class event_convert_test(object):
    """ 
    实例化该类以测试线程调整的性能。
    可以看到，event和flag的切换大概都花费了0.15ms的时间，因此能用event就不要用flag，因为这样更加正宗！
    而直接处理可以达到1μs的时间，相差近100倍，因此从实时性上来说确实应该直接在回调中进行处理。
    不过，因为视觉反馈的数据大都在几十ms，亚毫秒的延迟是不产生任何影响的。所以，从编程的合理性上应该使用event。
    """
    def __init__(self,mode) -> None:
        rospy.init_node('test')
        self.mode = mode
        self.trance = Float32()
        self.pose_pub = rospy.Publisher("/ar",Float32,queue_size=10)
        self.trance.data = 0
        self.sub = rospy.Subscriber("/ar",Float32,self.except_producer,queue_size=1)
        self.event = Event()
        self.flag = False
        threading.Thread(target = self.pub,daemon=True).start()
        if self.mode == 0:
            threading.Thread(target = self.process_event,daemon=True).start()
        else: threading.Thread(target = self.process_flag,daemon=True).start()
        self.start = 0
        rospy.spin()

    def except_producer(self,message:Float32):
        start = time.time()
        self.message = message
        end = time.time()
        print('直接处理耗费时间为',end-start)
        self.start = time.time()
        self.flag = True
        if self.mode==0: self.event.set()

    def pub(self):
        times = 0
        self.pub.__dict__['thread'] = True
        while True:
            times+=1
            self.trance.data += 1
            self.pose_pub.publish(self.trance)
            rospy.sleep(0.5)
            if times == 6:break

    def process_event(self):
        while True:
            self.event.wait()
            message = self.message
            end = time.time()
            print('event处理耗费时间为',end-self.start)
            self.event.clear()

    def process_flag(self):
        while True:
            if self.flag == True:
                message = self.message
                end = time.time()
                print('flag处理耗费时间为',end-self.start)      
                self.flag = False          

class if_test(object):
    """ 可看到时间差距很小基本上可忽略 """
    def __init__(self,msg=None,name:str='None',data_class='None',):
        from typing import Optional
        import cv2
        self.pub_topics = {'None':1}
        s = time.time()
        if None not in [name,data_class]:
            if not self.pub_topics.get(name):  # 首次处理进行发布者创建
                pass
            a=1
        else: return msg
        e = time.time()
        t1= e-s
        print("经历if时间为：",t1)
        s = time.time()
        e = time.time()
        t2= e-s
        print("什么都没时间为：",t2)
        print("二者时间差距为：",t1-t2)

class list_param_test(object):
    ''' 可以看到默认参数不是函数每次运行重新赋值，而是在预处理阶段只赋值（引用）了一次，后续就是引用了 '''
    def __init__(self) -> None:
        self.list_test(1)
        self.list_test(0)
    def list_test(self,a,param=[1,2]):
        if a==1: param[0] = 2
        print(param)

class exec_eval_test(object):
    def __init__(self) -> None:
        """ 
        可以看到，在if判断较少的情况下，eval比exec慢了几倍，而exec比if判断慢了10倍。
        所以能用if就用if，不要用字符串执行。
        """
        a = np.uint8([0,0])
        a = cv2.bitwise_not(a)  # 假如去掉这行代码，即不将a先经过OpenCV转换一下格式，会惊奇的发现t4-t1小于0。
        start = time.time()
        a = cv2.bitwise_not(a)
        end = time.time()
        t1 = end - start
        # eval方式
        n = 'not'
        start = time.time()
        a = eval(f'cv2.bitwise_{n}(a)')
        end = time.time()
        t2 = end - start
        print('eval额外消耗的时间为',t2-t1)
        # exec方式
        n = 'not'
        start = time.time()
        exec(f'a=cv2.bitwise_{n}(a)')
        end = time.time()
        t3 = end - start
        print('exec额外消耗的时间为',t3-t1)
        # if方式
        start = time.time()
        a = self.__bitwise_X(a,'not')
        end = time.time()
        t4 = end - start
        print('if判断额外消耗的时间为',t4-t1)

    def __bitwise_X(self,a,operator,b=None):
        if operator == 'and':
            a = cv2.bitwise_and(a,b)
        elif operator == 'or':
            a = cv2.bitwise_or(a,b)
        elif operator == 'xor':
            a = cv2.bitwise_xor(a,b)
        elif operator == 'not':
            a = cv2.bitwise_not(a)
        return a

class def_test(object):
    def __init__(self) -> None:
        start = time.time()
        end = time.time()
        t1 = end - start
        start = time.time()
        def test():
            i=0
            i+=1
        end = time.time()
        t2 = end -start
        print(t2-t1,'可见，定义一个函数基本上不消耗任何时间')
        test()

class read_grab_test(object):
    """ read一次大概在2-3ms，而grab则只需要几十微妙，相差2个数量级。 """
    def __init__(self) -> None:
        cap = StdVideo.Cap()
        time.sleep(0.5)
        start = time.time()
        cap.read()
        end = time.time()
        print('read消耗的时间为',end-start)
        time.sleep(0.5)
        start = time.time()
        cap.grab()
        end = time.time()
        print('grab消耗的时间为',end-start)

class read_jump_test(object):
    """ 可以看到，第二幅图是倾斜后图，证明从cap中读到的始终是最新的图，也就是是说cap并没有对图像进行任何缓存 """
    def __init__(self) -> None:
        cap = StdVideo.Cap()
        ret,frame1 = cap.read()
        print('现在倾斜一下身体，以便检查后续图片是否为最新')
        time.sleep(3)
        ret,frame2 = cap.read()
        cv2.imshow('1',frame1)
        cv2.imshow('2',frame2)
        cv2.waitKey(0)

class cv_show_test(object):
    def __init__(self) -> None:
        """
        可以看到，必须关闭旧窗口后，新的设定才生效。另外，如果删除一个不存在或已经删除了的window会报错。
        另外，尽管这里没有测试，但要注意，一旦删除了某个图像又新创建，不会恢复此前建立的与该窗口有关的联系。
        """
        cap = StdVideo.Cap()
        ret,frame = cap.read()
        cv2.namedWindow('2',cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('2',cv2.WINDOW_NORMAL)
        cv2.imshow('2',frame)
        print('只有删除之前创建的新的才生效')
        cv2.waitKey(0)
        cv2.namedWindow('2',cv2.WINDOW_NORMAL)
        cv2.destroyWindow('2')
        cv2.namedWindow('2',cv2.WINDOW_AUTOSIZE)   
        cv2.imshow('2',frame)
        cv2.destroyWindow('2')
        print('只有删除之前创建的新的才生效')
        cv2.waitKey(0)
        cv2.namedWindow('2')
        cv2.destroyWindow('2')
        try:cv2.destroyWindow('2')
        except:print('错误：删除不存在的窗口')
        cv2.namedWindow('2',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('2',200,300)
        cv2.imshow('2',frame)
        print('对normal使用resize可以确定初始状态不影响拖动')
        cv2.waitKey(0)
        cv2.destroyWindow('2')
        cv2.namedWindow('2',cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow('2',200,300)
        cv2.imshow('2',frame)
        print('对auto使用resize会导致图像窗口可以拖动，但是图像不会随之变化')
        cv2.waitKey(0)

# 尝试二级实例化关联不太行
class CapClass(object):
    def __init__(self,device) -> None:
        self.device  = device
        self.cap = cv2.videocapture(device)
    def read(self):
        self.frame = self.cap.read()

    class WindowClass(object):
        def __init__(self,window_name,frame) -> None:
            self.window = window_name
            self.frame = frame
        def draw(self):
            pass

    def __call__(self,window_name,frame) -> WindowClass:
        return CapClass.WindowClass(window_name,frame)

class assign_test(object):
    """
    可以看到，引用赋值的消耗的时间与数据的大小没有什么关系，因为这是引用关系，本质上都是赋值了地址，时间在亚微秒级，是很快的。
    而非引用的copy方式，则差距明显，因为需要copy一下，并且差距可达2个数量级至多。
    而数字这种哈希值的赋值则根据大小不同，其要复制的哈希表也不同，因此时间会不同，可达1个数量级之多,比copy要好一些。
    总结就是：能不copy就不copy，即，能引用就引用（这是可变类型的好处；图像作为ndarray就是一种可变类型，其就是引用赋值）。
    """
    def __init__(self) -> None:
        a = [1,2]
        b = [9 for _ in range(9999)]
        # 引用赋值
        start = time.time()
        c = a
        end = time.time()
        print(type(c),end-start)
        start = time.time()
        d = b
        end = time.time()
        print(type(d),end-start)
        # 非引用赋值
        start = time.time()
        c = a.copy()
        end = time.time()
        print(type(c),end-start)
        start = time.time()
        d = b.copy()
        end = time.time()
        print(type(d),end-start)
        # 哈希赋值
        a = "9"
        b = "999999999999999999999999999999999999999"
        start = time.time()
        c = a
        end = time.time()
        print(type(c),end-start)
        start = time.time()
        d = b
        end = time.time()
        print(type(d),end-start)

def auto_node_init():
    import subprocess
    from sensor_msgs.msg import Image
    import atexit
    # 通过roscore启动rosmaster
    p = subprocess.getoutput("pgrep rosmaster")
    if p == '':
        subprocess.Popen('roscore',start_new_session=False)
        def safe_exit():
            subprocess.getoutput("kill %s " % subprocess.getoutput("pgrep rosmaster"))
            print('正在关闭rosmaster')
        atexit.register(safe_exit)
    print('正在启动rosmaster')
    time.sleep(5)  # 等待roscore启动完
    print('成功启动rosmaster')
    # 初始化节点
    rospy.init_node('test')
    try:
        rospy.wait_for_message("camera/color/image_raw",Image,timeout=1)
    except: print('超时')

def plt_show():
    # bind = StdVideo.Draw.mouse_draw(thickness=20)
    # 一行代码实现视频输出
    frames = []
    for _ in range(5):
        frames.append(StdVideo.Read())
    StdVideo.plt_imgs_show(frames,new_thread=True)
    print('等待')
    time.sleep(1)
    print('over')

def ndim_test():
    """ 可以看到，两者的速度都在10倍微秒级，但是ndim比判断可迭代快4倍 """
    from typing import Iterable
    a=1
    start = time.time()
    isinstance(a,Iterable)
    end = time.time()
    print(end-start)
    start = time.time()
    np.ndim(a)
    end = time.time()
    print(end-start)

def shape_dim_test():
    """ 可以看到，获取维度比获取shape更快，因为shape需要知道每一维度的元素量 """
    b = np.ones((1920,1080,3))
    start = time.time()
    b.shape
    end = time.time()
    print(end-start)
    start = time.time()
    b.ndim
    end = time.time()
    print(end-start)

def list_nparray_test():
    """ cnts是个list类型 """
    frame = StdVideo.Read()
    frame = StdVideo.ToBinary(frame,120,255,convert_color=True)
    cnts = StdVideo.ContourFinder.find(frame)
    print(type(cnts))

def split_slice_test():
    """ 可见split的速度确实不如numpy数组切片 """
    from numpy import ndarray
    # StdVideo.hsv_determine(2)
    StdVideo.Cap(2)
    while True:
        frame:ndarray = StdVideo.Read(2,resize_xyc=(1280,720))
        start = time.time()
        b,g,r = frame[:,:,0],frame[:,:,1],frame[:,:,2]
        end = time.time()
        print(end-start)
        start = time.time()
        b,g,r = cv2.split(frame)
        end = time.time()
        print(end-start)
        StdVideo.Show('output',[b,g,r],wait_time=1)
        break

def cap_resize_bug():
    """ 1920*1080 和 640*480均正常显示，但是1280*720显示颜色不正常，应是相机问题 """
    StdVideo.Cap(2,wh_set=(1920,1080))
    while True:
        start = time.time()
        frame = StdVideo.Read(2)
        end = time.time()
        print(1/(end-start))
        StdVideo.Show('output',frame,wait_time=1,window_size=None,window_mode=StdVideo.Types.WINDOW_AUTOSIZE)

def window_judge():
    """ 待try比不带try会慢一些， """
    cv2.namedWindow('ok')
    start = time.time()
    cv2.getWindowImageRect('ok')
    end = time.time()
    print(end-start)
    start = time.time()
    cv2.getWindowProperty('ok',0)
    end = time.time()
    print(end-start)
    start = time.time()
    try:
        cv2.getWindowImageRect('okk')
    except: pass
    end = time.time()
    print(end-start)
    start = time.time()
    try:
        cv2.getWindowProperty('okk',0)
    except: pass
    end = time.time()
    print(end-start)

# if __name__ == "__main__":
#     cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
#     # cap.set(3,1920)
#     # cap.set(4,1080)
#     while True:
#         start = time.time()
#         _,frame = cap.read()
#         print(frame.shape)
#         cv2.imshow('out',frame)
#         end = time.time()
#         print(1/(end-start))
#         key = cv2.waitKey(1) & 0xFF
#         if key == 27:break
#         # StdVideo.Show('output',frame,wait_time=1,window_size=(640,320),show_fps=True)

class test1():
    # 均值滤波
    def _feedback_average_smooth(self,nums,near_dis=0.001)->bool:
        """ 在对相机帧率要求不高，甚至需要降低帧率的时候，使用均值滤波以达到更稳定的效果 """
        if not hasattr(self._feedback_average_smooth,'times'):
            self._feedback_average_smooth.__dict__['times']=0
            self.__sum = np.zeros((6,nums))
        times = self._feedback_average_smooth.__dict__['times']
        self.__sum[:,times] = np.array(self.feedback_target_position+self.feedback_target_euler)
        print(self.__sum[:,times])
        self._feedback_average_smooth.__dict__['times']+=1
        if self._feedback_average_smooth.__dict__['times'] == nums:
            sum_:np.ndarray = np.sum(self.__sum,axis=1)/nums
            temp = sum_.tolist()
            self.feedback_target_position, self.feedback_target_euler = temp[:3],temp[3:6]
            self._feedback_average_smooth.__dict__['times']=0
            print(self.feedback_target_position+self.feedback_target_euler)
            return True
        return False

def key_test():
    import sys,tty,termios,select
    while True:
        start = time.time()
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0001)
        if rlist:
            key = sys.stdin.read(2)  # sys.stdin.read() returns a string on Linux
        else: key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if chr(3) in key: exit("Ctrl-C 按下，程序退出")
        elif 'p' in key: exit("Ctrl-C 按下，程序退出")
        end = time.time()
        print(end-start)
        if key != '': print(key)

def moveit_test():
    arm = MoveGroupCommander('arx5_arm')
    robot = RobotCommander()
    print(robot.get_link_names('arx5_arm'))
    print(arm.get_end_effector_link())

def q_to_e(q):
    print(tf_conversions.transformations.euler_from_quaternion(q))

def numpy_end_test():
    """ 可以看到，取-1比取某个特定的索引慢了近一个数量级 """
    arr = np.arange(1000)
    time1 = time.time()
    arr[-1]
    time2 = time.time()
    print(time2-time1)
    time1 = time.time()
    arr[567]
    time2 = time.time()
    print(time2-time1)

# # 从管道中获取帧
# frames = pipeline.wait_for_frames()
# # 获取RGB图像帧
# color_frame = frames.get_color_frame()
# color_image = StdVideo.ToCV2(color_frame)

# import exifread
# with open('/home/ghz/img.jpg','rb') as f:
#     tags = exifread.process_file(f)
# print(tags)
# FL = tags.get('EXIF FocalLength')
# FL = FL.values[0]
# DZR = tags.get('EXIF DigitalZoomRatio')

# CLOSE_CMD = 'EB 90 01 05 10 F4 01 64 00 6F'
# OPEN_CMD = 'EB 90 01 03 11 F4 01 0A'
# VID_PID = '6790_29987'
# command = bytes.fromhex(OPEN_CMD)
# StdVideo.ToSerial(None,VID_PID)

# import asyncio

# event = asyncio.Event()
# event.wait()
# event.set()
# event.is_set()

# que = asyncio.Queue()
# que.put(1)
# que.get()

# asyncio.run()
# asyncio.create_task()
# asyncio.wait()

# async def go():
#     await asyncio.sleep(1)

# from multiprocessing import Queue,Pipe
# que = Queue(3)

# from colorama import init, Fore, Back, Style

# init(autoreset=True)

# class Log(object):

#     #  前景色:红色  背景色:默认
#     def red(self, s):
#         return Fore.RED + s + Fore.RESET

#     #  前景色:绿色  背景色:默认
#     def green(self, s):
#         return Fore.GREEN + s + Fore.RESET

#     #  前景色:黄色  背景色:默认
#     def yellow(self, s):
#         return Fore.YELLOW + s + Fore.RESET

#     #  前景色:蓝色  背景色:默认
#     def blue(self, s):
#         return Fore.BLUE + s + Fore.RESET

#     #  前景色:洋红色  背景色:默认
#     def magenta(self, s):
#         return Fore.MAGENTA + s + Fore.RESET

#     #  前景色:青色  背景色:默认
#     def cyan(self, s):
#         return Fore.CYAN + s + Fore.RESET

#     #  前景色:白色  背景色:默认
#     def white(self, s):
#         return Fore.WHITE + s + Fore.RESET

#     #  前景色:黑色  背景色:默认
#     def black(self, s):
#         return Fore.BLACK

#     #  前景色:白色  背景色:绿色
#     def white_green(self, s):
#         return Fore.WHITE + Back.GREEN + s

#     def dave(self, s):
#         return Style.BRIGHT + Fore.GREEN + s


# color = Log()
# print(color.red('I am red!'))
# print(color.green('I am gree!'))
# print(color.yellow('I am yellow!'))
# print(color.blue('I am blue!'))
# print(color.magenta('I am magenta!'))
# print(color.cyan('I am cyan!'))
# print(color.white('I am white!'))
# print(color.white_green('I am white green!'))
# print(color.dave("www.cndba.cn"))
# print(111)

def interpolate_test():
    from scipy.interpolate import make_interp_spline,CubicSpline
    import numpy as np

    # 生成两个测试数据
    x = np.array([0, 4])
    y = np.array([0, 4])

    # # 创建5次样条插值对象
    # bs = make_interp_spline(x, y, k=5,
    #     bc_type=(((1, 0), (2, 0)),((1, 0), (2, 0))))
    # 创建3次样条插值对象
    cs = CubicSpline(x, y, bc_type="clamped")
    bs = CubicSpline(x, y,)
    # 在新的x坐标上计算插值结果
    x_new = np.linspace(0, 4, 500)
    y_new5 = bs(x_new)
    y_new3 = cs(x_new)
    start = time.time()
    fs = make_interp_spline(x,y,k=5,bc_type=(((1, 0), (2, 0)),((1, 0), (2, 0))))
    fs(x_new)
    end = time.time()
    print(end-start)  # 时间差不多是在0.3ms，很快的
    return
    import matplotlib.pyplot as plt
    def plot(y,t,t_interp,y_interp,pause=0,clear=True,ion=False,block=False):
        """
        绘制样条插值的结果
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
            t_interp: 插值后的时间序列，一个一维数组
            y_interp: 插值后的时间序列对应的函数值，一个一维数组
        """
        if ion: plt.ion()  # 开启交互模式
        if clear: plt.clf()
        # 绘制原始数据点
        plt.plot(t, y,'r-',label='original',markersize=5)
        # 绘制样条插值的结果
        plt.plot(t_interp,y_interp,'g-',label='spline',markersize=3)
        # 添加图例和标签
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.ylabel('y')
        plt.title('Spline Interpolation')
        # 显示图形
        plt.show(block=block)
        if pause>0: plt.pause(pause)

    plot(y_new3,x_new,x_new,y_new5,block=True)

# import rospy
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from isaacgym import ShowImages

# class ROSImageSubscriber:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.show_images = ShowImages("ROS Image", (800, 600))
#         self.subscriber = rospy.Subscriber("/camera/image", Image, self.callback)

#     def callback(self, data):
#         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         images = {"ROS Image": cv_image}
#         self.show_images.show(images)

# def main():
#     rospy.init_node("ros_image_subscriber")
#     ros_image_subscriber = ROSImageSubscriber()
#     rospy.spin()

# if __name__ == "__main__":
#     main()

# from tf_conversions import transformations
# q = transformations.quaternion_multiply([-5, 6, 7, 8],[0, 0, 0, 1])
# print(transformations.euler_from_quaternion([0.5,0.5,0.5,0.5]))