#!/usr/bin/env python3

import math
import rospy
from actionlib import SimpleActionServer
import arx5_msgs.msg

from scipy.interpolate   import CubicSpline,make_interp_spline,interp1d
from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from typing import List,Union

import matplotlib.pyplot as plt
import numpy as np
import functools
import time
import tkinter as tk
from threading import Thread

# 滑条调参
min_vel = 0.4
def parameter_test():
    # 回调函数
    def on_slider_change(val, slider_name):
        global min_vel
        if slider_name == 'Vel_Min':
            min_vel = int(val)/1000
            print('min_vel:',min_vel,'\r\n')

    root = tk.Tk()
    root.title("Parameter Tester")
    max_value = 6000
    min_value = 100
    slider_data = [
        {'name': 'Vel_Min', 'init_value': min_value},
    ]

    for data in slider_data:
        slider_name = data['name']
        init_value = data['init_value']
        # 创建滑动条
        slider = tk.Scale(
            root,
            from_=min_value,
            to=max_value,
            orient=tk.HORIZONTAL,
            command=lambda val, name=slider_name: on_slider_change(val, name),
            label=slider_name,
            length=600
        )
        slider.pack(padx=10, pady=10)
        # 设置滑条初始值
        slider.set(init_value)

    # 创建退出按钮
    exit_button = tk.Button(root, text="Exit", command=root.quit)
    exit_button.pack(padx=10, pady=10)
    root.mainloop()
# Thread(target=parameter_test,daemon=True).start()  # 调试用

class MoveItAction(object):
	# TODO：这两个类目前还没有怎么用起来
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()
    CONTROL_MODE = ['torque','position']  # 控制模式选择
	# Action initialisation
    def __init__(self,name,control_mode,interpolation=5,plot=False):
        """
            name：action服务器的名字（在simple_moveit_controllers.yaml和ros_controllers.yaml中定义）。
            interpolation目前支持：1为线性插值；2-5为n次样条插值。
        """
        self.drag = rospy.get_param("~drag", default=False)
        self.cheat_target = None
        self.interpolation = interpolation
        self.t_delta = 0.005  # 单位:s
        self.plot = plot
        self.arm_joint_nums = 6
        self.gripper_joint_nums = 0
        self.all_joint_nums = self.arm_joint_nums + self.gripper_joint_nums
        self.init_pose = [0,-0.025,0.025,0,0,0]  # 初始状态（单位为rad，不全为零，防止干涉认定导致规划不稳定）

        self.new_action = False
        self.times = 0; self.max_times = 100

        self.cmd_publisher = rospy.Publisher('/arx5/joint_command', arx5_msgs.msg.JointCommand, queue_size=10)
        self.cmd = arx5_msgs.msg.JointCommand()
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.header.frame_id = 'arx5'
        self.cmd.mode = control_mode
        self.cmd.kp = [100 for _ in range(self.arm_joint_nums)]
        self.cmd.kd = [10 for _ in range(self.arm_joint_nums)]
        self.cmd.position = self.init_pose
        self.cmd.velocity = [0.8 for _ in range(self.arm_joint_nums)]  # 初始化速度值23度/s，从而控制电机缓慢移动到0位
        self.cmd.effort = [0 for _ in range(self.arm_joint_nums)]
        self.cmd.id = [id for id in range(1,self.arm_joint_nums+1)]

        rospy.Timer(rospy.Duration(1. / 200.), self.control_continue)
        rospy.Timer(rospy.Duration(1. / 50.), self.send_joint_cmd)

        self._action_server = SimpleActionServer(name,FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._action_server.start()

	# Action callback
    def execute_cb(self,goal_handle:Union[FollowJointTrajectoryGoal,list]):
        # 避免函数被重复运行（ROS多个话题发布时，某个订阅者的回调函数将会被开启多次，即便上次的回调函数还没执行完）
        if not hasattr(self.execute_cb,'running'):
            self.execute_cb.__dict__['running'] = False
        if self.execute_cb.__dict__['running']: return
        else: self.execute_cb.__dict__['running'] = True

        # 不同类型处理（本质上都是获得统一的points变量）
        if isinstance(goal_handle,list):  # 此时为一个6+6+1长度的列表
            points = [JointTrajectoryPoint(),JointTrajectoryPoint()]
            points[0].time_from_start = rospy.Duration(0)
            points[0].positions = goal_handle[:6]
            points[1].time_from_start = goal_handle[12]
            points[1].positions = goal_handle[6:12]
        else: points:List[JointTrajectoryPoint] = goal_handle.trajectory.points  # 获得目标途径上的所有轨迹点（无任何外在约束的情况下一般只有两个点）

        # 构造初始矩阵
        times = len(points)
        time_array = np.zeros(times)  # 关节位置的时间信息列表
        joints_matrix = np.zeros((times,self.arm_joint_nums))  # 存储关节位置插值数据用的列表
        # 分别获得各个点的时间信息和位置信息，放到两个列表中
        for time,point in enumerate(points):  # 单位为rad；每个point对应了某个时间点
            time_array[time] = point.time_from_start.to_sec()  # 不同关节具有相同的时间点
            joints_matrix[time] = point.positions  # point.positions[j]存储了关节j在当前时间的角度值；joints_matrix每一行存储了某个时间点所有关节的角度目标

        # 选择是否进行插值
        if rospy.get_param("/arx5/no_interpolate", default=0) == 0:
            # 轨迹插值
            execute_time_array = Interpolate.time_clip(0,time_array[times-1],self.t_delta)  # 将轨迹首尾点的时间以5ms来分段，与rospy.Rate(200)对应上
            new_times = len(execute_time_array)
            if isinstance(self.interpolation,int):
                if self.interpolation == 1:  # 线性插值
                    interpolate = functools.partial(Interpolate.linear_interpolate,t=time_array,t_i=execute_time_array,plot=self.plot)
                elif self.interpolation == 3:  # 三次B样条插值
                    interpolate = functools.partial(Interpolate.cubic_spline,t=time_array,t_i=execute_time_array,plot=self.plot)
                elif 1 < self.interpolation <= 5:  # 五次B样条插值
                    interpolate = functools.partial(Interpolate.spline,t=time_array,t_i=execute_time_array,k=self.interpolation,plot=self.plot)
                else: exit('暂不支持该类型的插值方式')
            else: exit('暂不支持该类型的插值方式')
            joints_matrix_new = np.apply_along_axis(interpolate,axis=0,arr=joints_matrix)  # axis=0表示对各个列施加函数
            # 速度计算
            speed_matrix = np.apply_along_axis(Interpolate.linear_speed_calculate,axis=0,arr=joints_matrix_new,t=self.t_delta)
        else:  # 不进行插值
            joints_matrix_new = joints_matrix
            new_times = times
            # 速度计算
            speed_matrix = np.apply_along_axis(Interpolate.linear_speed_calculate,axis=0,arr=joints_matrix_new,t=time_array)
        joints_list = joints_matrix_new.tolist()

        # 速度限幅
        min_vel = rospy.get_param("/arx5/min_vel", default=0)
        if min_vel == 0: min_vel = 2.593
        else: rospy.set_param("/arx5/min_vel",0)
        max_vel = rospy.get_param("/arx5/max_vel", default=0)
        if max_vel == 0: max_vel = 2*math.pi
        else: rospy.set_param("/arx5/max_vel",0)
        speed_min_limit_matrix = np.abs(speed_matrix) + min_vel  # 对速度求绝对值，因为底层仅接收正速度；限制最小速度
        speed_list:list = np.where(speed_min_limit_matrix>max_vel,max_vel,speed_min_limit_matrix).tolist()  # 限制最大速度
        speed_list.insert(0,list(speed_list[0]))  # 发送第一个点的期望速度设置为与第二个点相同

        # 遍历所有插值时间发送cmd
        self.new_action = True
        while self.continue_ok: pass
        r = rospy.Rate(200)  # 200Hz，5ms控制频率
        self.cheat_target = rospy.get_param('/arx5/cheat_target',default=None)
        for i in range(new_times):  # 遍历每一个插值点
            # 若有新的action中断发生，则立刻终止当前处理，转而下一个处理
            if self._action_server.is_preempt_requested():
                self.new_action = False
                self._action_server.set_preempted()
                return
            # 控制命令发布（只发6个关节的，即cmd长度为6）
            self.cmd.velocity = speed_list[i]
            self.cmd.position = joints_list[i]
            self.cmd.header.stamp = rospy.Time.now()
            if self.cheat_target is None:
                self.cmd_publisher.publish(self.cmd)
            else: self.cheat_publish()  # 发布虚假命令
            r.sleep()  # 这种sleep是动态调整的，以保证尽可能地按照频率进行执行
        self.new_action = False
        self.times = 0

	    # 设置结构通知moveit执行完毕
        self._result.error_code = 0
        self._action_server.set_succeeded(self._result)
        self.execute_cb.__dict__['running'] = False

    def control_continue(self,event):
        """ 接收到控制消息后，发送完扔继续发送一段时间最后的目标值，防止丢包造成位置误差增大 """
        self.continue_ok = False
        if not self.new_action:
            self.cmd.header.stamp = rospy.Time.now()
            if self.drag: self.cmd.effort = [1 for _ in range(self.arm_joint_nums)]
            if self.cheat_target is None: self.cmd_publisher.publish(self.cmd)
            else: self.cheat_publish()
            self.continue_ok = True
            if self.times >= self.max_times:
                self.times = 0
                if self.cheat_target is None:
                    self.new_action = True
                self.max_times = 20  # 初始化为100，因为初始化的时候由于启动不同步问题容易丢包；之后都是20；
            else: self.times += 1
        else:  # 不发送阶段检测param
            self.cheat_target = rospy.get_param('/arx5/cheat_target',default=None)
            if self.cheat_target is not None: self.new_action = False

    def send_joint_cmd(self,event):
        """ 定时检查是否采用不经过Moveit规划进行控制（但任然用到了逆解得到的关节角度值，通过param传递） """
        joint_target = rospy.get_param('/arx5/joint_target',None)
        if joint_target is not None:
            rospy.set_param('/arx5/joint_target',None)
            joint_target = [float(x) for x in joint_target.strip('[]').split(',')]  # 字符串解码为list（列表长度为6+6+1）
            self.execute_cb(joint_target)

    def test(self,joints_list,time_array,speed_list,times,enable=False):
        """ 打印一些参数 """
        if enable:
            loginfo(f'起止点为：{joints_list[0]} {joints_list[1]}')
            loginfo(f'时间间隔：{time_array}')
            loginfo(f'速度值为：{speed_list}')
            loginfo(f'点数量为：{times}')

    def cheat_publish(self):
        """ 将控制命令发布转换为关节状态发布（相当于光速执行） """
        if not hasattr(self.cheat_publish,'first'):
            self.cheat_publish.__dict__['first'] = False
            from sensor_msgs.msg import JointState
            self.cheat_pub = rospy.Publisher("/arx5/joint_states", JointState, queue_size=2)
            self.state_eq_target = JointState()
            self.state_eq_target.name = ["base_link_to_link1","link1_to_link2","link2_to_link3","link3_to_link4","link4_to_link5","link5_to_gripper_link1"]
            self.state_eq_target.header.frame_id = "arx5"
        if self.cheat_target == 1:  # 发布接收到的命令值
            self.state_eq_target.position = self.cmd.position
            # self.state_eq_target.velocity = self.cmd.velocity
            # self.state_eq_target.effort   = self.cmd.effort
            self.state_eq_target.header.stamp = self.cmd.header.stamp
        elif self.cheat_target == 0:
            states = rospy.get_param('/arx5/cheat_joints_state',default='[0,0,0,0,0,0]')
            self.state_eq_target.position = [float(x) for x in states.strip('[]').split(',')]
            self.state_eq_target.header.stamp = rospy.Time.now()
        self.cheat_pub.publish(self.state_eq_target)

class Interpolate(object):
    """ 轨迹插值 """
    @staticmethod
    def time_clip(start,end,interval,unit='s',end_control=False):
        """ms级的时间细化
            unit表示所给三个时间的单位
            end_control
        """
        if unit == 's': precision = 0.001
        elif unit == 'ms': precision = 1
        time_line = (np.array([start,end,interval])/precision).astype('int32')
        time_clipped = np.arange(time_line[0],time_line[1],step=time_line[2])
        if end_control:
            if time_clipped[-1] != time_line[1]:
                time_clipped = np.append(time_clipped,time_line[1])
        else: time_clipped = np.append(time_clipped,time_line[1])
        time_clipped = time_clipped.astype('float64')
        time_clipped*=precision
        return time_clipped

    @classmethod
    def linear_interpolate(cls,y,t,t_i,unit='s',sort=False,plot=False):
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t,y = t[idx],y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i,(float,int)):
            flag = True
            t_interp = cls.time_clip(t[0],t[-1],t_i,unit)
        else:
            flag = False
            t_interp = t_i
        # 使用LinearNDInterpolator进行线性插值
        y_interp = interp1d(t,y)(t_interp)
        if plot: cls.plot(t,y,t_interp,y_interp)
        # 返回插值后的时间序列和函数值
        if flag: return t_interp, y_interp
        else: return y_interp

    @classmethod
    def cubic_spline(cls,y,t,t_i,unit='s',bc_type='clamped',sort=False,plot=False):
        """
        使用 UnivariateSpline 实现 1-5次样条插值，常用3次和5次
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
            ti:
            unit: 时间单位是s还是ms
            bc_type: 'not-a-knot','natural'，'clamped'
        返回值:
            返回一个二元组，包含插值后的时间序列和对应的函数值
        """
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t,y = t[idx],y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i,(float,int)):
            flag = True
            t_interp = cls.time_clip(t[0],t[-1],t_i,unit)
        else:
            flag = False
            t_interp = t_i
        # 使用CubicSpline进行3次样条插值
        cnt = 1
        while True:
            try: y_interp = CubicSpline(t,y,bc_type=bc_type)(t_interp)
            except: t[-cnt] += 0.0003/cnt  #  Expect x to not have duplicates or x must be strictly increasing sequence.
            else: break
            cnt += 1
        if plot: cls.plot(t,y,t_interp,y_interp)
        # 返回插值后的时间序列和函数值
        if flag: return t_interp, y_interp
        else: return y_interp

    @classmethod
    def spline(cls,y,t:np.ndarray,t_i,k=5,unit='s',sort=False,plot=False):
        """
        使用 make_interp_spline 实现 1-5次样条插值，常用3次和5次
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
        返回值:
            当t_i为间隔时，返回一个二元组，包含插值后的时间序列和对应的函数值
            当t_i为时间序列时，仅返回插值后的函数值
        """
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t,y = t[idx],y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i,(float,int)):  # 指定频率
            flag = True
            t_interp = cls.time_clip(t[0],t[-1],t_i,unit)
        else:  # 指定已经细化的时间点
            flag = False
            t_interp = t_i
        # 使用 make_interp_spline 进行 5次B样条插值
        cnt = 1
        while True:
            try: y_interp = make_interp_spline(t,y,k=k,bc_type=(((1, 0), (2, 0)),((1, 0), (2, 0))))(t_interp)
            except: t[-cnt] += 0.0003/cnt  #  Expect x to not have duplicates or x must be strictly increasing sequence.
            else: break
            cnt += 1
        if plot: cls.plot(t,y,t_interp,y_interp)
        # 返回插值后的时间序列和函数值
        if flag: return t_interp, y_interp
        else: return y_interp

    @staticmethod
    def linear_speed_calculate(y:np.ndarray,t):
        """ 计算曲线两点间的直线变化率 """
        y_cp = y.copy()
        y1 = np.delete(y,0)
        y2 = np.delete(y_cp,-1)
        if isinstance(t,(np.ndarray,list,tuple)):
            return (y1-y2)/(t[1:]-t[:-1])
        else: return (y1-y2)/t

    @staticmethod
    def interval_limit(y_i:np.ndarray,min_delta=0.001,max_delta=3):
        """ 控制插值后y方向的间隔 """
        end_index = y_i.shape[0] - 1
        i = 1
        # 中间元素限幅
        while i < end_index:
            if math.fabs(y_i[i] - y_i[i-1]) < min_delta:
                y_i = np.delete(y_i,i)
                end_index -= 1  # 删除中间元素后有效长度-1（即有效长度始终记录的是0-原末尾元素的长度）
                y_i = np.append(y_i,y_i[end_index])
            else: i += 1  # 未删除元素计数+1
        # 末尾（原）两元素限幅
        if math.fabs(y_i[end_index] - y_i[end_index-1]) < min_delta:
            y_i[end_index-1] = y_i[end_index]
        return y_i

    @staticmethod
    def plot(t,y,t_interp,y_interp,pause=0,clear=True,ion=False,block=False):
        """
        绘制样条插值的结果:
            主要参数:
                t: 时间序列，一个一维数组
                y: 时间序列对应的函数值，一个一维数组
                t_interp: 插值后的时间序列，一个一维数组
                y_interp: 插值后的时间序列对应的函数值，一个一维数组
            辅助参数:
                block: 图片显示后是否阻塞
                pause: 图片显示后的延迟时间,block为False时生效
                clear: 本次图片是否覆盖上次图片
                ion: 是否开启交互模式
            默认是“不阻塞+无等待+覆盖+无交互”，用于实时显示最新的插值结果
        """
        if ion: plt.ion()  # 开启交互模式
        if clear: plt.clf()
        # 绘制原始数据点
        plt.plot(t,y,'ro',label='original',markersize=3)
        # 绘制样条插值的结果
        plt.plot(t_interp,y_interp,'g-',label='interpolated',markersize=3)
        # 添加图例和标签
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.ylabel('y')
        plt.title('Interpolation Result')
        # 显示图形
        plt.show(block=block)  # 非block并不会清除原来显示的图片，而是等下次图片来覆盖（前提是使用了plt.clf()）
        if pause > 0 and not block: plt.pause(pause)


if __name__ == '__main__':

    NODE_NAME = "follow_joint_trajectory_server"

    def loginfo(msg: str):
        rospy.loginfo("[{}] {}".format(NODE_NAME, msg))

    rospy.init_node(NODE_NAME)
    loginfo("Initializing {} node.".format(NODE_NAME))

    MoveItAction('arm_position_controller/follow_joint_trajectory',MoveItAction.CONTROL_MODE[1],interpolation=5,plot=False)

    rospy.loginfo("Ready to follow joint trajectory!!!!")
    rospy.spin()