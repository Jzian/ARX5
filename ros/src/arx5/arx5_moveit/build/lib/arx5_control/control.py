#!/usr/bin/env python3

# 初始化参数调整
PICK_UNDER_Z = -0.048
PICK_PITCH = -0.142
PICK_UNDER_J = (-0.012194361681285817, -2.2039296524966128, 0.8577801577948777, -0.08287603390175528, -0.01931941116887408, -0.012077165693382696)

try:  # 导入ROS包
    from moveit_commander import MoveGroupCommander,RobotCommander
    from moveit_msgs.msg import RobotState,Constraints,PositionConstraint,OrientationConstraint,TrajectoryConstraints,BoundingVolume
    from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from shape_msgs.msg import SolidPrimitive
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    import rospy
    import tf_conversions  # 不用ROS可以自行pip install tf-conversions（但是这里可能有冲突问题，默认会优先使用pip安装的库）
    from arx5_msgs.msg import JointCommand
except ImportError as e: print(f'导入ROS相关包出错:{e}。若不使用ROS可忽略。')
# 导入通用包
import numpy as np
import math,time
from typing import Optional,Union,List,Dict
from threading import Thread,Event
from copy import deepcopy,copy
import sys,os
import termios,tty,select  # 获取键盘值要额外用到的标准库
# from colorama import Fore, Back, Style  # 打印彩色文本，例如print(Fore.GREEN + Back.WHITE + 'This is green text on white background')


class RoboticArmAgent(object):
    """
        RAA类。初始化参数包括是否使用仿真、控制模式、是否自动跟随、是否自动夹取，以及初始位置、节点名
        初始位置init_pose参数：0Home姿态，1Standby姿态，2Forward姿态,3 ,None当前姿态
        node_name:
            默认为空字符''，此时表示外部不初始化ROS，由该类自动初始化，此时节点名为ARX5
            为None时表示不使用ROS，不会进行ROS初始化
            为非空字符串时表示外部初始化，此时其值应与外部节点名保持一致
    """
    def __init__(self,use_sim,control_mode='g_i',init_pose=None,node_name='',excute_mode=-1) -> None:
        self.__jiumin = 0
        # 根据节点名参数情况进行处理
        self.__ros_init(node_name)
        # 是否不允许执行
        self.set_excute_mode(excute_mode)
        # 初始化参数
        self.__param_init(use_sim)
        # 初始化控制
        self.__control_init(control_mode)
        # 初始化预置位置
        self.__preset_pose_init()
        # 初始化位置
        self.__start_pose_init_(init_pose)

    def __ros_init(self,node_name):
        """ 初始化ROS节点 """
        if node_name == '': self.node_name = 'ARX5'
        else: self.node_name = node_name
        if (self.node_name is not None) and not rospy.core.is_initialized():
            rospy.init_node(node_name)
            self.loginfo('Initializing......')
        elif self.node_name is None: print('节点名为None，不使用ROS')

    def __param_init(self,use_sim):
        """ 根据是实机还是仿真进行参数的不同初始化 """
        # 选择是仿真还是实机（有不同的参数和控制方式）
        self.use_sim = use_sim;self.loginfo(f'use_sim={self.use_sim}')
        self.target_pose = PoseStamped()
        self.wait_flag = True  # 初始化全局等待标志
        # 初始化新目标存储变量
        self.new_target_xyz,self.new_target_rpy  = [0.0,0.0,0.0],[0.0,0.0,0.0]
        # 与moveit控制精度、稳定性有关的变量（总体误差1mm，xyz控制精度至少应达到0.1mm，其分辨力至多为0.033mm）
        self.discrimination:int = 0  # 鉴别力阈，单位：0.1mm（适当增加可以提高稳定性，从而达到综合的较小误差;经实验，设置在0.3-0.5mm较为不错，否则稳定性误差可能将达到将近1mm；0表示不设置阈值）
        self.precise_pose = True  # 表示给定的pose是精确的值
        self.position_tolerance = 0.00005  # 0.05mm的目标的位置公差
        self.orientation_tolerance = 0.01  # 0.01rad的目标的角度公差
        self.joint_tolerance = 0.0017 # 0.1°的目标关节角度公差

    def __control_init(self,mode:str):
        """
            # TODO:许多mode是有待测试的(mode决定了目标数据处理和moveit控制的不同方式)
            注：self.target_base_mode='current'， self.start_base_mode=1这种情况是不合理的
        """
        if mode in ['traditional','td','t']:  # 实机可用
            self.use_integral = False   # 是否采用偏差积分的方式进行移动
            self.target_base_mode = 'current'  # 即target设定的基准是基于当前状态还是基于上次目标，前者必须要保证偏差量要大于重力影响
            self.start_base_mode  = 0  # 即moveit轨迹规划的起点位置是当前状态0还是上次目标1
        elif mode in ['no-gravity_integral','ng_i']:  # 实机可用
            self.use_integral = True
            self.target_base_mode = 'current'
            self.start_base_mode  = 0
        elif mode in ['gravity_integral','g_i']:  # SIM可用
            self.use_integral = True
            self.target_base_mode = 'last'
            self.start_base_mode  = 1
        elif mode in ['local_integral','l_i']:
            self.use_integral = True
            self.target_base_mode = 'last'
            self.start_base_mode  = 0
        elif mode in ['constant_exe','c_e']:
            self.use_integral = True
            self.target_base_mode = 'last'
            self.start_base_mode  = 1
            self.wait_flag = True  # 该模式下必须要求执行完

        if self.use_integral is True:
            self.discrimination:int = 0  # 采用积分方式时，实际上积分判断的函数中已经加入了微小偏差的忽略机制，因此这里可以设置为0
        self.loginfo('控制模式：'+mode)

        self.__moveit_init()
        self.__update_last_target(True)

    def __moveit_init(self,group_name='arx5_arm',reference_link=0):
        """ 初始化moveit接口(reference_link为0或-1，对应base_link和最末端的link) """
        # 初始化机器人接口
        robot = RobotCommander('/arx5/robot_description',ns='/arx5')
        # 防止同一个launch文件启动时，moveit接口还没启动好，导致报错退出
        while True:
            try: self.arm_moveit = MoveGroupCommander(group_name,robot_description='/arx5/robot_description',ns='/arx5',wait_for_servers=10)
            except Exception as e: self.loginfo(f'未找到Group或未连接到server，重新尝试中……{e}'),rospy.sleep(0.4)
            else: break
        # 得到活动关节名称及数量（不考虑虚拟的以及fixed）
        self.joints_name = self.arm_moveit.get_active_joints()
        self.joints_num = len(self.joints_name)
        # 坐标系配置
        self.links_name = robot.get_link_names(group_name)  # 所有坐标系
        self.end_effector_link = self.links_name[-1]  # 末端坐标系
        self.reference_frame = self.links_name[reference_link]  # 参考坐标系(默认为base_link)
        self.target_pose.header.frame_id = self.reference_frame  # 设置目标变量的参考系
        self.arm_moveit.set_pose_reference_frame(self.reference_frame)  # 设置pose的参考系
        # plan规划配置
        self.arm_moveit.allow_replanning(True)  # 这里的true与set_num_planning_attempts(1)并不完全矛盾(根据chatGPT)
        self.arm_moveit.set_planning_time(2)  # 经验证，可以为浮点数
        self.arm_moveit.set_num_planning_attempts(1) # 规划的尝试次数
        # 目标精度控制
        self.arm_moveit.set_goal_position_tolerance(self.position_tolerance)  # 设定位置误差允许值,m;0.001为1mm的误差之内
        self.arm_moveit.set_goal_orientation_tolerance(self.orientation_tolerance)  # 设定角度误差允许值,rad
        self.arm_moveit.set_goal_joint_tolerance(self.joint_tolerance)  # 即0.01度
        # 初始化为最大的速度和加速度
        self.arm_moveit.set_max_acceleration_scaling_factor(1.0) # 加速度限制
        self.arm_moveit.set_max_velocity_scaling_factor(1.0)     # 速度限制

    def __preset_pose_init(self):
        """
            初始化预置的pose(默认关节角度；标注e的为末端xyzrpy位姿)，可摆脱对moveit配置文件的依赖
            部分pose值仿真和实机不同
            使用print(self.preset_pose)查看所有预置关节空间pose
        """
        self.pick_place_0_1 = 0
        pick_real = (0,-1.5,1.187,-1.169,0,0)
        pick_under = PICK_UNDER_J
        self.preset_pose = {'Home':(0,-0.025,0.025,0,0,0),'Standby':(0,-0.419,0.314,0.297,0,0),"Forward":(0,-1.57,0.73,0.85,0,0),
                            'Middle':(2.271,-math.pi/2,1.169,-1.169,0,0),'η':(0,-1.623,1.047,-1.012,0,0),
                            'PickSim':(0,-1.62,1.047,-1.012,0,0),
                            'PickReal':pick_real,'PickUnder':pick_under,
                            'One':(0,-math.pi/2,math.pi,0,0,0),'Gravity':(0,-math.pi,math.pi,0,0,0),
                            'RPY0':(0,-0.1467,0.1935,1.524,0,math.pi)}
        if self.use_sim:  # 仿真
            self._pick_scan_xyz_dict = {0:[0.274,0,0.067891],1:[0.274,0,0.067891]}
            self._pick_rpy = (-math.pi,0,math.pi/2)
            self._pick_base_z = 0.035  # TODO: 与待夹取的物块的深度有关，在多层物块情况下，需要获得物块深度（如通过激光或图像面积区间判断物块的层数）这里暂时仅考虑单层情况
            self._place_xy = (-0.27,0)  # 放置时的基准位置
            self._place_rpy = (self._pick_rpy[0],self._pick_rpy[1],-math.pi/2)
        else:  # 实机
            self._pick_scan_xyz_dict = {0:[0.2845,0,PICK_UNDER_Z],1:[0.2845,0,PICK_UNDER_Z]}
            self._pick_rpy = (-3.124,PICK_PITCH,0)
            self._pick_base_z = self._pick_scan_xyz_dict[0][2] - 0.016  # 位控-0.022
            # self._pick_base_z = self._pick_scan_xyz_dict[0][2] - 0.01  # 力控
            # place的位置初始化，后续可以通过task_pick_place_param_init方法覆盖
            self._place_xy = (0.303,0.204)
            self._place_rpy = self._pick_rpy
        self._place_base_z = self._pick_base_z  # place的基础就是pick的基础
        # 组建完整pose
        self.prepose_pick_scan = tuple(self._pick_scan_xyz_dict[0]) + self._pick_rpy
        self.prepose_pick_base = (*self._pick_scan_xyz_dict[0][:2],self._pick_base_z) + self._pick_rpy
        self.prepose_place_base = (*self._place_xy,self._place_base_z) + self._place_rpy

    def __start_pose_init_(self,preset_pose,sleep_time=2):
        """ 到预置的位置(传入None表示保持当前位置) """
        if preset_pose == 0: self.go_to_named_or_joint_target('Home',sleep_time=sleep_time)
        elif preset_pose == 1: self.go_to_named_or_joint_target('Standby',sleep_time=sleep_time)
        elif preset_pose == 2: self.go_to_named_or_joint_target('Forward',sleep_time=sleep_time)
        elif preset_pose == 3: self.go_to_named_or_joint_target('η',sleep_time=sleep_time)
        elif preset_pose == 4: self.go_to_named_or_joint_target('One',sleep_time=sleep_time)
        elif preset_pose == 5: self.go_to_named_or_joint_target('Gravity',sleep_time=sleep_time)

    def gravity_test(self):
        """ 测试重力影响 """
        self.go_to_named_or_joint_target('Gravity',sleep_time=5)

    def set_reference_frame(self,mode=0):
        """ 设置参考坐标系（但实际效果似乎莫得） """
        if mode == 0:  # 设为底盘坐标系
            self.arm_moveit.set_pose_reference_frame('base_link')
        else:  # 设为末端坐标系
            self.arm_moveit.set_pose_reference_frame(self.arm_moveit.get_end_effector_link())

    def change_to_gripper_CS(self,xyz_rela):  # TODO
        # 求出yaw的偏差
        current_yaw_bias = -(self._pickplace_init_rpy[2] - self.current_rpy[2])  # 以顺时针偏转时该bias为正值代入后续计算
        # 根据当前的roll偏差解算出实际应该移动的前后和左右（z轴和y轴）距离（近似认为当前roll和反馈的数据是同步的）
        tf_trans_x =  xyz_rela[0]*math.cos(current_yaw_bias) - xyz_rela[1]*math.sin(current_yaw_bias)  # 左右移动
        xyz_rela[1] = xyz_rela[0]*math.sin(current_yaw_bias) + xyz_rela[1]*math.cos(current_yaw_bias)  # 前后移动
        xyz_rela[0] = tf_trans_x  # 临时保存的用完
        return xyz_rela

    def change_joints_to_pose(self,joints_value,print_flag=False):
        """ 根据关节角度计算得到末端执行器的位姿(注意返回的xyzw和rpy不是对应的，rpy的r和p进行了置换以满足实际感觉，而xyzw无法简单置换) """
        # rospy.wait_for_service('/compute_fk')  # 其实没必要等服务
        fk_service = rospy.ServiceProxy('/arx5/compute_fk',GetPositionFK)
        robot_state = RobotState()
        joint_state = JointState()
        joint_state.name = self.joints_name
        joint_state.position = joints_value
        robot_state.joint_state = joint_state

        fk_link_names = [self.end_effector_link]
        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = self.reference_frame
        fk_request.fk_link_names = fk_link_names
        fk_request.robot_state = robot_state

        fk_response = fk_service.call(fk_request)
        if fk_response.error_code.val == fk_response.error_code.SUCCESS:
            pose_stamped:PoseStamped = fk_response.pose_stamped[0]
            pose = pose_stamped.pose
            xyz = [pose.position.x,pose.position.y,pose.position.z]
            xyzw = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            rpy = list(tf_conversions.transformations.euler_from_quaternion(xyzw))
            if print_flag:
                print(f"End effector position(xyz): {xyz}")
                print(f"End effector orientation(xyzw):{xyzw}")
                print(f"End effector euler(rpy):{rpy}")
            return xyz,rpy,xyzw
        else: exit(f"FK service failed with error code: {fk_response.error_code.val}")

    def change_pose_to_joints_value(self,pose:Union[PoseStamped,list],precise_mode=True)->List[float]:
        """ 将pose转换为joint角度目标 """
        if isinstance(pose,list):
            pose_temp = PoseStamped()
            pose_temp.pose.position.x = pose[0]
            pose_temp.pose.position.y = pose[1]
            pose_temp.pose.position.z = pose[2]
            if len(pose) == 6:
                xyzw = tf_conversions.transformations.quaternion_from_euler(pose[3],pose[4],pose[5])
            else: xyzw = pose[3:7]
            pose_temp.pose.orientation.x = xyzw[0]
            pose_temp.pose.orientation.y = xyzw[1]
            pose_temp.pose.orientation.z = xyzw[2]
            pose_temp.pose.orientation.w = xyzw[3]
            pose = pose_temp
        # 临时借用
        self.arm_moveit.set_joint_value_target(pose,self.end_effector_link,precise_mode)
        joints_value = self.arm_moveit.get_joint_value_target()
        # 恢复原来
        self.arm_moveit.clear_pose_targets()
        self.arm_moveit.set_joint_value_target(self.target_pose,self.end_effector_link,precise_mode)
        return joints_value
 
    def _change_list_to_target_pose(self,xyz:Optional[list]=None,rot:Optional[list]=None,update_mode=2,global_protect=False):
        """
            将姿态列表转换为target_pose：
                update_mode决定了只关心更新xyz(0)、rpy(1)，还是全部更新(2)，默认为2全部更新。
                若不给定某个list，则自动通过相应的全局pose_list进行配置，然后返回None(基于xyz和xyzw)。
            允许改动的全局list只有self.target_pose_position和self.target_pose_euler。
            self.__target_pose_euler_displace和self.__target_pose_orientation只能通过特定函数自动更新。
            global_protect为true则不更改全局变量，而是将列表转换后返回一个新的pose对象(仅支持update_mode=2)。
        """
        if not global_protect:  # 修改全局，返回None
            if update_mode != 1:
                if xyz is not None:
                    self.target_pose.pose.position.x,self.target_pose.pose.position.y,self.target_pose.pose.position.z = xyz
                    self.target_pose_position = xyz
                else: self.target_pose.pose.position.x,self.target_pose.pose.position.y,self.target_pose.pose.position.z = self.target_pose_position
            if update_mode != 0:
                if rot is not None:
                    if len(rot) == 3:
                        self.target_pose_euler = rot
                        self.target_pose_orientation = list(tf_conversions.transformations.quaternion_from_euler(*rot))
                    else:
                        self.target_pose_orientation = rot
                        self.target_pose_euler = list(tf_conversions.transformations.euler_from_quaternion(rot))
                    self.target_pose.pose.orientation.x,self.target_pose.pose.orientation.y,self.target_pose.pose.orientation.z,self.target_pose.pose.orientation.w = self.target_pose_orientation
                else:
                    self.target_pose.pose.orientation.x,self.target_pose.pose.orientation.y,self.target_pose.pose.orientation.z,self.target_pose.pose.orientation.w = self.target_pose_orientation
        else:  # 不修改全局，仅仅返回值
            pose_temp = PoseStamped()
            pose_temp.pose.position.x,pose_temp.pose.position.y,pose_temp.pose.position.z = xyz
            if len(rot) == 3:
                xyzw = tf_conversions.transformations.quaternion_from_euler(*rot)
            else: xyzw = rot
            pose_temp.pose.orientation.x,pose_temp.pose.orientation.y,pose_temp.pose.orientation.z,pose_temp.pose.orientation.w = xyzw
            return pose_temp

    def _change_target_pose_to_list(self,target_pose:Optional[PoseStamped]=None,global_protect=False):
        """
            将target_pose转换为pose列表
            target_pose为none时直接调整全局姿态变量，无返回值；否则，返回给定的姿态转换后的list量，且完成全局变量的更新。
            list量分为三部分：xyz，正常顺序的rpy，以及调整顺序rpy
        """
        if target_pose is None:  # 基于去全局更新
            self.target_pose_position = [self.target_pose.pose.position.x,self.target_pose.pose.position.y,self.target_pose.pose.position.z]
            self.target_pose_orientation = [self.target_pose.pose.orientation.x,self.target_pose.pose.orientation.y,self.target_pose.pose.orientation.z,self.target_pose.pose.orientation.w]
            self.target_pose_euler = list(tf_conversions.transformations.euler_from_quaternion(self.target_pose_orientation))
        else:  # 基于给定更新
            target_pose_pose_position = [target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z]
            target_pose_pose_oritation = [target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w]
            target_pose_pose_euler = list(tf_conversions.transformations.euler_from_quaternion(target_pose_pose_oritation))
            if not global_protect:  # 不保护全局
                self.target_pose_position = target_pose_pose_position
                self.target_pose_euler = target_pose_pose_euler
                self.target_pose_orientation = target_pose_pose_oritation
                self._change_list_to_target_pose()  # 基于给定target影响全局list
            else: return target_pose_pose_position,target_pose_pose_euler # 若保护则返回转换后的列表

    def __set_start_state_to_last_joints_target(self)->None:
        """ 设定起始状态为上次的目标（克服重力影响） """
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joints_name # active不包含virtual joint ["base_link_to_link1","link1_to_link2","link2_to_link3","link3_to_link4","link4_to_link5","link5_to_gripper_link1"] #  
        joint_state.position = self.last_joint_target  # 初始值在gotonamed中赋值，后续均在设置moveit目标后更新
        # print(joint_state.name , joint_state.position)  # 调试用
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.arm_moveit.set_start_state(moveit_robot_state)

    def __set_start_state_to_current_state(self)->None:
        """ 设定起始状态为当前的状态（不考虑重力影响） """
        self.arm_moveit.set_start_state_to_current_state()

    def __set_given_target_pose(self,target_pos:Optional[list],target_rot:Optional[list]=None,target_base:Union[str,int]='last')->None:
        """
            设定单一目标的基准和相对偏差，得到绝对的最终目标变量
            目标姿态分为两部分：基准值和相对值。基准值的设定通过target_base_mode参数配置，共有三种配置：
                'current':当前姿态 'last':上次的目标值 'zero':0,从而相对值实际上意义变成了绝对姿态
                为避免出错，三者分别可以简写为c l z('0',0)
                一般增量方式（包括积分方式）采用last（推荐）或current，绝对移动采用0
                默认为last模式，因为last不受重力以及是否停稳的影响，是应首选的方式（跟踪时自动做好了失败回退）
            target_xyz,target_rpy中有一个为None时相应的目标pose不会进行任何改动，相当于可以方便独立实现xyz或rpy(xyzw)的控制,
            但允许target_xyz为6/7个元素，则后3/4个元素为旋转角度，此时rpy/xyzw的参数值将被覆盖。
        """
        # 目标基准确定
        if target_base in ['current','c']:
            self.get_current_state()
            target_base_xyz,target_base_xyzw = self.current_xyz,self.current_xyzw
        elif target_base in ['zero','z','0',0]:
            target_base_xyz,target_base_xyzw = (0,0,0),(0,0,0,1)  # 按xyzw的单位(恒等)四元数
        elif target_base in ['last','l']:
            target_base_xyz,target_base_xyzw = self.last_xyz,self.last_xyzw
        # 最终目标确定
        if target_pos is not None:
            self.target_pose_position = [target_base_xyz[0]+target_pos[0], target_base_xyz[1]+target_pos[1], target_base_xyz[2]+target_pos[2]]
            if len(target_pos) == 6: target_rot = target_pos[3:6]
            elif len(target_pos) == 7: target_rot = target_pos[3:7]
        if target_rot is not None:
            if len(target_rot) == 3:  # 将欧拉角转换为四元数
                self.target_pose_euler = target_rot
                target_rot = tf_conversions.transformations.quaternion_from_euler(*target_rot)
            else: self.target_pose_euler = list(tf_conversions.transformations.euler_from_quaternion(target_rot))
            # 四元数乘法计算目标rot
            self.target_pose_orientation = list(tf_conversions.transformations.quaternion_multiply(target_rot,target_base_xyzw))  # 基准四元数与目标四元数相乘，得到新的四元数
        # 维护姿态列表
        self._change_list_to_target_pose()

    def __update_last_target(self,first=False):
        """ 更新目标位置，该函数用在成功规划执行后，也即：__go_to_pose_target、set_and_go_to_way_points和go_to_named_or_joint_target三个函数中 """
        if first:  # 首次时将last初始化为current
            self.last_target_pose = self.get_current_state(get_pose=True)
            self.last_xyz = copy(self.current_xyz)
            self.last_rpy = copy(self.current_rpy)
            self.last_joint_target = copy(self.current_joints)
            self.last_xyzw = copy(self.current_xyzw)
            # 将初始化位置也作为一个当前目标
            self.target_pose = copy(self.last_target_pose)
            self.target_pose_position = copy(self.current_xyz)
            self.target_pose_euler = copy(self.current_rpy)
            self.target_pose_orientation = copy(self.current_xyzw)
            self.loginfo(f'初始化位姿为{self.current_xyz}+{self.current_rpy}+{self.current_xyzw}')
        else:
            self.last_target_pose = deepcopy(self.target_pose)
            self.last_xyz = copy(self.target_pose_position)
            self.last_rpy = copy(self.target_pose_euler)
            self.last_xyzw = copy(self.target_pose_orientation)
            self.last_joint_target = copy(self.new_joint_target)

    def __set_moveit_pose_target(self,precise_mode=True,target=None)->None:
        """
            通过moveit接口设定目标pose（这才是实际上的最终的目标设定，上面那个只是设置了一个变量而已而没有作用于moveit）
            target=None始终如此，只有在__go_to_pose_target
        """
        # ******目标设定*********(不论目标是一个或多个，这里有用的只有最后一个)
        try:
            if target is None:
                self.arm_moveit.set_joint_value_target(self.target_pose,self.end_effector_link,precise_mode)
            else: self.arm_moveit.set_joint_value_target(target,self.end_effector_link,precise_mode)
        except Exception as e: self.loginfo(f'目标设定失败：姿态超出限制范围：{e}'),self.arm_moveit.clear_pose_targets();return False
        else: self.new_joint_target = self.arm_moveit.get_joint_value_target();return True  # moveit仅会记录一个最新的target

    def set_wait_tolerance(self,tolerance:float):
        """ 设置执行完标志的角度偏差容许度。范围0.1(5.72度)-1，单位rad """
        min_tl,max_tl = 0.1,1  # 最小容许度应以实际电机为准
        if tolerance > max_tl:
            self.__wait_tolrerance = max_tl
            print(f'容许公差不得超出{max_tl}rad，已自动限幅到{max_tl}rad')
        elif tolerance < min_tl:
            self.__wait_tolrerance = min_tl
            print(f'容许公差不得小于{min_tl}rad，已自动限幅到{min_tl}rad')
        else: self.__wait_tolrerance = tolerance

    def __wait_excute(self,wait_flag:Union[bool,float]):
        """
            wait_flag=True时根据当前关节状态和目标状态判断是否执行完。
            wait_flag为大于0的浮点数时，wait_flag可设置为以秒为单位的延时值。
            wait_flag=False或0时，不进行延时。
        """
        if wait_flag == True:
            self.loginfo('动作执行中......')
            while True:
                self.get_current_state()
                if np.max(np.abs(np.array(self.current_joints) - np.array(self.new_joint_target))) <= self.__wait_tolrerance: break
                else: rospy.sleep(0.01)
            self.loginfo('执行完毕')
        elif wait_flag: rospy.sleep(wait_flag)

    def set_move_duration(self,time):
        """disable_plan前需要先设置这里的持续时间
        控制动作执行经历的时间；本机械臂的电机的转速最低都可以满足在1s内电机转动一圈，而机械臂通常最大活动范围在半圈，也就是说0.5s内机械臂可以到达几乎任何姿势。
        因此，当动作执行时间time>0.5时，总是会限制电机执行速度；而当time>0.25且关节移动小于1/4圈时，也同样会限制电机执行速度，但对于大于1/4圈的关节电机将达最大转速。
        特别地，当动作执行时间为0时，将直接给定电机目标而不进行插值，从而实现最快的到达。
        需要注意的是，当角度偏差（目标-当前/上次）和控制频率一定时，time越大意味着插值后的关节角度间隔越细，这反映到电机执行上有两个特点：
            1.电机在到达目标前被给定的target值的次数很多。
            2.电机在未接收到下个5ms的目标时，已经达到了这次的目标。
            以上两点意味着电机将像“微积分”般经过“无数次”“一顿一顿”地移动到目标位置。
            因此，要想运动的慢和想丝滑的移动是矛盾的，可以想象有这样一个折中点，即在5ms内，电机本次的目标没有执行完，而是执行了x%，此时，电机的速度较慢，同时也没有明显的卡顿。
            一个显然的注意点是，当限制位姿改变的速度一定时，意味着任何目标关节角度都将被拆分成同样细的间隔（间隔相同，角度越大，执行的次数越大，意味着经历的时间越大，符合t=x/v）。
                从而，以速度为限制对象，可以保证相同速度下不同目标姿态转换时电机运动丝滑度的一致性，但无法控制不同姿态转换经历的时间；相反，时间控制无法保证不同姿态转换时的丝滑程度，
                但是可以保证两个姿态转换的时间始终满足要求。通常来说，我们往往关注的重点不是对执行时间有严格要求，而是要求机械臂以一定速度稳定运行，因此，常约束机械臂的执行速度。
                并且，这个执行速度有一个保证机械臂稳定运行的下限，而其上限则是电机硬件的最大速度限制。
                而一般来说，速度不是瞬间达到的，而是由一定加速度斜坡式达到目标值的，因此，在加速的初期，必然会出现卡顿现象，且加速度约小，卡顿越明显。
                moveit对加速度的控制，本质上也只是作用于时间，根据S=0.5at^2(初速度为0)，以及v<=vmax，可以通过分段作图直观看到，当a减小时，达到最大速度也越慢，完成
                相同路程所经历的时间也约长。假设用于规划的目标还是只有首尾两点，那么插值的函数的形状都是一样的，不同的是曲线的时间尺度。显然，时间尺度越大，曲线的初始和末尾阶段也就被拉的
                越长，也就相当于加速过程越长，也即相当于反映了加速度限制的影响。不过，上述按照的是匀加速的方式，至于moveit是采用匀加速还是变加速暂时还不得而知，但本质上
                是相似的。
            综上所述，要想机械臂稳定运行，应尽可能地提高加速度和速度上限，以使轨迹的全过程的5ms电机的目标执行度<=x%。
            然而，考虑到机械臂除了运行较慢会卡顿抖动外，运行过快也可能会造成抖动，同时有时我们不希望移动太快。因此，加速度和速度的上限亦不能过大，
                所以需要进行折中选取速度和加速度的值，兼顾稳定和速度。
        """
        if time == 0: rospy.set_param("/arx5/no_interpolate",2)  # 最快速执行
        else: self.move_duration = time

    def set_move_speed(self,min,max):
        """ 全局速度限制(控制电机转速的上下限) #TODO """
        return

    def joints_speed_control(self,min_=None,max_=None):
        """
            机械臂全局的执行速度控制（控制电机底层转速，单位：rad）
            实际接收方是在follow_joint_trajectory_server.py中
            设置是一次性的，动作执行完后将恢复为默认配置（max_vel=360deg/s，min_vel=2.593）
        """
        if min_ is not None: rospy.set_param("/arx5/min_vel",min_)
        if max_ is not None: rospy.set_param("/arx5/max_vel",max_)

    def set_position_constraint(self,constraint):
        """
        设定运动过程中的空间约束（即轨迹必须在某个特定空间内运动）。constraint由以下几个部分组成：
            0基准：'base_link'
            1形状：BOX = 1，SPHERE = 2，CYLINDER = 3，CONE = 4
            2尺寸：(a,b,c)
            3姿态：(x,y,z,r,p,y)
            4对象：'gripper_link'
            5名字：'any_name'
            综上：constraint = ['base_link',(x,y,z),1,(a,b,c),(x,y,z,r,p,y),'gripper_link']
        """
        # 确定约束空间范围的位姿
        pose = PoseStamped()
        pose.header.frame_id = self.arm_moveit.get_planning_frame()
        pose.pose.position.x = constraint[3][0]
        pose.pose.position.y = constraint[3][1]
        pose.pose.position.z = constraint[3][2]
        rpy = [constraint[3][4],constraint[3][3],constraint[3][5]]
        xyzw = tf_conversions.transformations.quaternion_from_euler(*rpy)
        pose.pose.orientation.x = xyzw[0]
        pose.pose.orientation.y = xyzw[1]
        pose.pose.orientation.z = xyzw[2]
        pose.pose.orientation.w = xyzw[3]
        # 确定空间形状与尺寸
        bbox_prism = SolidPrimitive()
        bbox_prism.type = constraint[1]
        bbox_prism.dimensions = constraint[2]
        # 确定空间位置与姿态
        bbox = BoundingVolume()  # 用于约束机械臂的规划的允许的空间
        bbox.primitives = [bbox_prism]  # 确定空间范围
        bbox.primitive_poses = [pose.pose]  # 确定物体的位姿
        # 创建位置约束
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = constraint[0]
        position_constraint.link_name = constraint[4]
        position_constraint.constraint_region = bbox
        position_constraint.weight = 1.0
        constraints = Constraints()
        constraints.name = constraint[5]
        constraints.position_constraints.append(position_constraint)
        self.arm_moveit.set_path_constraints(constraints)

    def set_oritation_constraint(self,constraint):
        """
        设定运动过程中的空间约束（即轨迹必须在某个特定空间内运动）。constraint由以下几个部分组成：
            0基准：'base_link'
            1姿态：(r,p,y)
            2对象：'gripper_link'
            3公差：(x,y,z)
            4名字：'any_name'
            综上：constraint = ['base_link',(x,y,z),1,(a,b,c),(x,y,z,r,p,y),'gripper_link']            
        """
        # Create an orientation constraint for the right gripper
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = constraint[0]
        orientation_constraint.link_name = constraint[2]
        rpy = [constraint[1][1],constraint[1][0],constraint[1][2]]
        xyzw = tf_conversions.transformations.quaternion_from_euler(*rpy)
        orientation_constraint.orientation.x = xyzw[0]
        orientation_constraint.orientation.y = xyzw[1]
        orientation_constraint.orientation.z = xyzw[2]
        orientation_constraint.orientation.w = xyzw[3]
        orientation_constraint.absolute_x_axis_tolerance = constraint[3][0]
        orientation_constraint.absolute_y_axis_tolerance = constraint[3][1]
        orientation_constraint.absolute_z_axis_tolerance = constraint[3][2]
        orientation_constraint.weight = 1.0
        # Create a contraints list and give it a name;Append the constraint to the list of contraints
        constraints = Constraints()
        constraints.name = constraint[4]
        constraints.orientation_constraints.append(orientation_constraint)
        # Set the path constraints on the right_arm
        self.arm_moveit.set_path_constraints(constraints)

    def set_vel_acc_maxvalue(self,v_m=None,a_m=None):
        """ 设置关节运动的最大速度和加速度 """
        if v_m is not None: self.arm_moveit.set_max_velocity_scaling_factor(v_m)
        if a_m is not None: self.arm_moveit.set_max_acceleration_scaling_factor(a_m)

    def clear_constraints(self):
        self.arm_moveit.clear_path_constraints()

    def set_excute_mode(self,mode=-1):
        """
            mode：
                0时在rviz中不进行插值直接到达目标点；
                1时为在rviz中模拟真机或仿真进行插值移动；
               -1时为通常对IsaacSim和真机的控制模式（不需额外设置param）；
               -2时为临时无插值；
               -3时为全局无插值；
            可重复设置，不同模式可以叠加或覆盖。
        """
        self.__excute_mode = mode
        if mode == 0:
            rospy.set_param('/arx5/cheat_target',0)
        elif mode == 1:
            rospy.set_param('/arx5/cheat_target',1)
        elif mode == -2:  # 临时无插值
            rospy.set_param("/arx5/no_interpolate",1)
        elif mode == -3:  # 全局无插值
            rospy.set_param("/arx5/no_interpolate",2)
        elif mode == -4:  # 全局无plan
            pass
        elif mode == -5:  # 临时无plan
            pass
        rospy.sleep(0.5)

    def set_excute_target(self,target=0):
        """ 设置执行的对象（实机&仿真0或rviz1） """
        self.__excute_target = target
        rospy.set_param('/arx5/cheat_target',target)

    def cheat_excute(self):
        """ 虚假的执行（相当于无控制，直接改变关节状态） """
        rospy.set_param('/arx5/cheat_joints_state',f'{self.new_joint_target}')

    def no_plan_excute(self):
        pass

    def go_to_named_or_joint_target(self,target:Union[str,list,dict],sleep_time:float=0,vel_limit=1.0,acc_limit=1.0,start_base=1,info='',wait_flag=True,return_enable=False,no_interpolate=False,disable_plan=False):
        """
            到指定名称或指定关节角度（可任意关节）的目标
            start_mode=0设置起始为当前，否则为last
            该函数不受工作空间target和last_target问题的影响，但会施加影响，即根据运动学正解改变target的值，因而也可以用于纠正一些错 # TODO: 尝试在规划失败后自动使用该方式移动到standby姿态补救（或者执行成功后立刻中断，然后重新执行原目标）
        """
        self.arm_moveit.set_max_acceleration_scaling_factor(acc_limit) # 加速度限制
        self.arm_moveit.set_max_velocity_scaling_factor(vel_limit)     # 速度限制
        # ******起始设定*****
        if start_base == 0 or self.wait_flag==False:  # 若上次的等待标志不为true，则起始位置强制设置为当前位置，否则将会无法规划
            self.__set_start_state_to_current_state()
            self.wait_flag = wait_flag
            start_joints = self.current_joints
        else:
            self.__set_start_state_to_last_joints_target()
            start_joints = self.last_joint_target
        # 设置目标角度
        if isinstance(target,dict):
            target_temp = {}
            for key,value in target.items():
                target_temp[self.joints_name[key]] = value
            target = target_temp
        elif isinstance(target,str): target = self.preset_pose[target]
        self.arm_moveit.set_joint_value_target(target)  # 指定角度目标（list时规定全部关节位置，dict时可以控制任意关节）
        # 获取joint目标状态，从而某些情况下可以用来作为下一次规划的起始状态
        self.new_joint_target = self.arm_moveit.get_joint_value_target()
        if no_interpolate: rospy.set_param("/arx5/no_interpolate",1)
        # 不进行规划，直接通过反解得到的关节空间目标执行（速度较快，但大幅度姿态变换时容易发生机械臂自身的干涉碰撞问题）
        if disable_plan:
            rospy.set_param('/arx5/joint_target',str(start_joints + self.new_joint_target + [self.move_duration]))
        elif self.__excute_mode == 0: plan_success = True
        # 工作空间规划
        else: plan_success, traj, planning_time, error_code = self.arm_moveit.plan()  # error_code只在成功时有用好像，所以是没啥用的
        if plan_success:  # 只用go而不用plan+excute则无法
            self.arm_moveit.clear_pose_targets()  # It is always good to clear your targets after planning with poses
            if self.__excute_mode != 0:
                self.arm_moveit.stop()  # 避免多余动作Stop the current execution, if any
                self.arm_moveit.execute(traj,wait=wait_flag)
            else: self.cheat_excute()
            self.wait_flag = wait_flag  # 更新全局waitflag
            if info != '': self.loginfo(info)
            # 运动学正解设置pose目标值
            self.target_pose_position, self.target_pose_euler,self.target_pose_orientation = self.change_joints_to_pose(self.new_joint_target)
            self._change_list_to_target_pose()
            # 更新last为当前
            self.__update_last_target()
            if sleep_time > 0: rospy.sleep(sleep_time)
            return True
        # 若失败则结束程序或返回false
        else:
            self.arm_moveit.clear_pose_targets()  # It is always good to clear your targets after planning with poses 
            if return_enable: return False
            else: raise Exception(f"{self.node_name}程序退出：{target} Pose Failed")

    def go_to_any_joint_target(self,joint,inc_value,sleep_time:float=0,vel_limit=1.0,acc_limit=1.0,start_base=0,target_base='l',info='',wait_flag=True,return_enable=False,no_plan=False):
        """
            任意数量关节移动函数封装；joint从机械臂底部向末端对应0-max；增量单位为rad；joint可以为int或list
            target_mode与工作空间类似，可以设置为current、last和0三种，前两种是增量式移动，后一种为绝对式移动
        """
        new_joints_target:Dict[list] = {}  # 使用字典方式指定目标关节
        joints_name = self.arm_moveit.get_active_joints()
        # 单关节控制
        if isinstance(joint,int):
            if target_base in ['l','last']:
                new_target = self.last_joint_target[joint]
            elif target_base in ['c','current']:
                new_target = self.arm_moveit.get_current_joint_values()[joint]
            else: new_target = 0
            # 赋值
            if isinstance(inc_value,(list,tuple)):
                if target_base in [0,'0','zero']:
                    new_joints_target[joint] = inc_value  # 非增量式移动
                else:
                    new_joints_target[joint] = []
                    for v in inc_value:
                        new_target += v
                        new_joints_target[joint].append(new_target)
            else:
                new_joints_target[joint] = inc_value + new_target
        # 多关节控制(此时joint为list)
        else:
            # 设置角度基准值（三种模式）
            if target_base in ['l','last']:
                joint_base = copy(self.last_joint_target)
            elif target_base in ['c','current']:
                joint_base = self.arm_moveit.get_current_joint_values()
            else: joint_base = [0 for _ in range(self.joints_num)]
            # 赋值
            for i,j in enumerate(joint):
                if isinstance(inc_value[i],(list,tuple)):  # 关节的角度值是个列表，表示多值绝对或累加移动
                    if target_base in [0,'0','zero']:
                        new_joints_target[j] = inc_value[i]  # 非增量式移动
                    else:  # 增量值移动
                        new_joints_target[j] = []
                        for v in inc_value[i]:
                            joint_base[j] += v
                            new_joints_target[j].append(joint_base[j])
                else: new_joints_target[j] = inc_value[i] + joint_base[j]
        # 设定目标并执行
        self.go_to_named_or_joint_target(new_joints_target,sleep_time,vel_limit,acc_limit,start_base,info,wait_flag,return_enable,disable_plan=no_plan)

    def go_to_shift_single_axis_target(self,axis:int,inc_value:float,target_base:Union[str,int]='last',vel_limit=1.0,acc_limit=1.0,sleep_time=0,info='',no_plan=False):
        """ 单轴增量式移动函数封装。注意该函数是给定的目标值是增量值不是绝对值。绝对值请用：go_to_single_axis_target函数 """
        return self.go_to_shift_multi_axises_target({axis:inc_value},target_base,vel_limit,acc_limit,sleep_time,info,no_plan=no_plan)

    def go_to_shift_multi_axises_target(self,axises_value:Dict[int,float],target_base:Union[str,int]='last',vel_limit=1.0,acc_limit=1.0,sleep_time=0,info='',no_plan=False):
        """
            多轴增量式移动函数封装
            给定轴-值对来进行多轴移动,如{0:0.025};target_base_mode可以是'current'或'last'
            该函数基本上相当于：self.set_and_go_to_pose_target([],[],target_base_mode)将不需移动的设置为0即可。
        """
        if info != '': print(info)
        target_temp = [0 for _ in range(6)]
        for key,value in axises_value.items(): target_temp[key] = value
        if target_base in ['zero','z','0',0]: raise Exception('增量式移动目标基准模式不能配置为0')
        succeed = self.set_and_go_to_pose_target(target_temp[:3],target_temp[3:6],target_base,vel_limit=vel_limit,acc_limit=acc_limit,sleep_time=sleep_time,no_plan=no_plan)
        return succeed

    def go_to_single_axis_target(self,axis,target,target_ref:str='last',vel_limit=1.0,acc_limit=1.0,sleep_time=0,info='',no_plan=False):
        """ 单轴绝对式移动函数封装。注意该函数是给定的目标值是绝对值不是增量值。增量值请用： go_to_shift_single_axis_target """
        if info != '': print(info)  # 可以打印一些信息
        self.arm_moveit.set_max_acceleration_scaling_factor(acc_limit) # 加速度限制
        self.arm_moveit.set_max_velocity_scaling_factor(vel_limit)     # 速度限制
        self.arm_moveit.clear_pose_targets()  # It is always good to clear your targets after planning with poses
        # 获得基准(不论哪种模式均通过直接修改target_pose的方式完成)
        if target_ref in ['current','c']:
            self.get_current_state()
            self.target_pose_position,self.target_pose_euler = list(self.current_xyz), list(self.current_rpy)  # 使用当前的姿态作为基准
        elif target_ref not in ['last','l']: raise Exception('单轴移动目标基准模式不能配置为0')
        # 单轴修改
        if axis<3: self.target_pose_position[axis] = target; self._change_list_to_target_pose(update_mode=0)
        else: self.target_pose_euler[axis-3] = target; self._change_list_to_target_pose(rot=self.target_pose_euler,update_mode=1)
        # 执行动作
        self.__go_to_pose_target(self.start_base_mode,wait_flag=True,retry_times=10,sleep_time=sleep_time,disable_plan=no_plan)

    def __go_to_pose_target(self,start_base,wait_flag=True,precise_mode=True,retry_times=10,show_plan_time=False,return_enable=False,constraints=None,disable_plan=False,sleep_time=0):
        """
            到指定pose（其中有更新上次的目标关节状态；可以指定“路点”到达；该函数是其它所有工作空间规划的函数的底层实现），而该函数不能独立使用。
            retry_times默认为10次尝试。重新设置目标值然后进行plan与moveit自身配置的重新规划是有差别的，前者更好，因为可以重算逆解。
            return_enable表示当N次尝试均失败后是直接退出还是返回False。通常，在动态跟踪时使能return；另外，对于某些特定的姿态，为了控制的精确性和方便性，
                往往设置了一个预置的关节空间的命名姿态，以及对应的工作空间的姿态（二者因重力影响存在较大偏差）。一般来说，工作空间的规划容易失败，
                因此可以使能return，在规划失败时，先通过关节空间执行到附近，然后再进行工作空间的规划执行。
        """
        attempt_times = 1
        precise_mode_cp = precise_mode
        # 设定轨迹约束
        if constraints is not None: self.arm_moveit.set_path_constraints(constraints)
        # 规划与执行以及失败处理
        while True:
            # ******起始设定*****
            if start_base == 0 or self.wait_flag==False:  # 若上次的等待标志不为true，则起始位置强制设置为当前位置，否则将会无法规划
                self.__set_start_state_to_current_state()
                self.wait_flag = wait_flag
                start_joints = self.current_joints
            else:
                self.__set_start_state_to_last_joints_target()
                start_joints = self.last_joint_target
            # 设定moveit的目标pose
            if not self.__set_moveit_pose_target(precise_mode): exit('逆解超范围，程序退出，请检查姿态目标是否合理')
            # 不进行规划，直接通过反解得到的关节空间目标执行（速度较快，但大幅度姿态变换时容易发生机械臂自身的干涉碰撞问题）
            if disable_plan:
                rospy.set_param('/arx5/joint_target',str(start_joints + self.new_joint_target + [self.move_duration]))
                plan_success = True
            elif self.__excute_mode == 0: plan_success = True  # 不进行规划，执行
            # 工作空间规划
            else: plan_success, traj, planning_time, error_code = self.arm_moveit.plan()  # error_code只在成功时有用好像，所以是没啥用的

            self.arm_moveit.clear_pose_targets()  # It is always good to clear your targets after planning with poses
            # 执行
            if plan_success:
                if attempt_times != 1 and precise_mode_cp:  # 若此前有规划失败，则规划成功后先调整精确模式，然后再规划后执行
                    precise_mode = True
                    attempt_times = 1
                    self._change_target_pose_to_list()  # 在失败尝试go_to_named_or_joint_target更新了list，因此需要重新修改回来，否则update_last_target()时将不正确
                else:
                    if disable_plan: pass  # disable_plan和self.__excute_mode很类似，都是不经过执行
                    elif self.__excute_mode != 0:
                        if show_plan_time: self.loginfo(f"规划成功，用时为{planning_time*1000}ms，共尝试{attempt_times}次。开始执行!")
                        self.arm_moveit.stop()  # 避免多余动作Stop the current execution, if any
                        self.arm_moveit.execute(traj,wait=wait_flag)  # 因为上面调用了stop，故理论上无需等待，否则stop将不起作用必然将等待完成；但是不等待机械臂会明显表现出卡顿，因为plan频率过低。
                    else: self.cheat_excute()
                    self.__update_last_target()  # 执行成功才更新last
                    if sleep_time > 0: rospy.sleep(sleep_time)
                    return True  # plan成功退出函数
            # 失败处理
            else:
                if not retry_times:
                    self.loginfo(f"{self.node_name}:MoveIt规划失败{error_code}目标关节角度值:{self.new_joint_target}")
                    if return_enable:
                        self.loginfo('未允许多次尝试，返回False')
                        return False  # 如果没有启用loop则直接退出，否则继续循环直到plan成功
                    else: exit(f'{self.node_name}未设置多次尝试、未使能返回，故程序直接退出')
                else:
                    if return_enable:
                        self.loginfo('多次尝试均规划失败，返回False')
                        return False  # 如果没有启用loop则直接退出，否则继续循环直到plan成功
                    else: exit('多次尝试规划失败，未使能返回，故程序直接退出')

    def set_and_go_to_pose_target(self,target_pos:Optional[list],target_rot:Optional[list]=None,target_base:Union[str,int]=0,sleep_time=0,wait_flag=True,start_base=None,return_enable=False,constraints=None,vel_limit=1.0,acc_limit=1.0,no_plan=False):
        """
            设定单一目标的基准和相对偏差，得到绝对的最终目标变量
            目标姿态分为两部分：基准值和相对值。基准值的设定通过target_base_mode参数配置，共有三种配置：
                'current':当前姿态 'last':上次的目标值 'zero':0,从而相对值实际上意义变成了绝对姿态
                为避免出错，三者分别可以简写为c l z('0',0)
                一般增量方式（包括积分方式）采用last（推荐）或current，绝对移动采用0
                默认为last模式，因为last不受重力以及是否停稳的影响，是应首选的方式（跟踪时自动做好了失败回退）
            target_xyz,target_rpy中有一个为None时相应的目标pose不会进行任何改动，相当于可以方便独立实现xyz或rpy(xyzw)的控制,
            但允许target_xyz为6/7个元素，则后3/4个元素为旋转角度，此时rpy/xyzw的参数值将被覆盖。
        """
        self.arm_moveit.set_max_acceleration_scaling_factor(acc_limit) # 加速度限制
        self.arm_moveit.set_max_velocity_scaling_factor(vel_limit)     # 速度限制
        self.__set_given_target_pose(target_pos,target_rot,target_base)
        if start_base is None: start_base = self.start_base_mode  # 设置为全局模式
        succeed = self.__go_to_pose_target(start_base,wait_flag=wait_flag,precise_mode=True,return_enable=return_enable,constraints=constraints,sleep_time=sleep_time,disable_plan=no_plan)
        return succeed

    def set_and_go_to_way_points(self,way_points:list,start_base=1,target_base:Union[str,int]=0,vel_limit=1.0,acc_limit=1.0,sleep_time=0,retry_times=10,wait_flag=True,info_front='',info_end=''):
        """
            设定笛卡尔路点列表并移动。way_points是一个列表，其中包含要经过的pose，格式为[x,y,z,r,p,y]。有三种模式：
                last和current模式表示目标值的起始值，然后目标值均是增量。并且这种增量是按序依次移动的，而不是始终基于初始值。这比较符合大多数的时候人的逻辑。
                0模式则表示所有的坐标都是绝对坐标。#TODO：目前仅支持0模式
            使用路点的方便性在于可以比较丝滑地一次性执行多段轨迹。
            缺点在于，路点的过程中的速度、加速度限制各段都是一样的，不能进行更加精细的控制。并且规划成功率最低(大距离移动时必须要细化路点)。
        """
        if info_front != '': print(info_front)
        self.arm_moveit.set_max_acceleration_scaling_factor(acc_limit) # 加速度限制
        self.arm_moveit.set_max_velocity_scaling_factor(vel_limit)     # 速度限制
        # ******起始设定*****
        if start_base == 0: self.__set_start_state_to_current_state()
        else: self.__set_start_state_to_last_joints_target()
        # 推荐传入参数直接是pose，这样就免去了进一步的处理
        if isinstance(way_points[0],PoseStamped):
            way_points = [point.pose for point in way_points]  # 需要转换为pose类型
        elif isinstance(way_points[0],(list,tuple)):  # 传入的是列表或元组类型的目标位置
            way_points = [self._change_list_to_target_pose(point[:3],point[3:6],global_protect=True).pose for point in way_points]
        # 非绝对时要对waypoints进行一波叠加处理，使所有的点均变成绝对点
        if target_base in ['c','current']:pass
        elif target_base in ['l','last']:pass
        # 终点维护
        self.target_pose.pose = way_points[-1]
        self._change_target_pose_to_list()
        # 插入waypoints起点为当前位置，保证轨迹正确执行（因为第一个点应该尽可能接近当前位置否则容易失败）
        if start_base == 0: way_points.insert(0,self.get_current_state(get_pose=True).pose)
        else: way_points.insert(0,self.last_target_pose.pose)
        retry_cnt = 0
        while True:
            # 逆解维护
            self.__set_moveit_pose_target()  # 这一步主要是为了更新joint_new，从而可以self.__update_last_target()
            self.arm_moveit.clear_pose_targets()  # 清空在上面函数中设定的目标值，因为实际规划没用它
            # 轨迹规划
            if self.__excute_mode != 0:
                traj,fraction = self.arm_moveit.compute_cartesian_path(way_points,eef_step=0.01,jump_threshold=0)
            else: fraction = 1
            if fraction == 1:
                if self.__excute_mode != 0:
                    self.arm_moveit.stop()  # 避免多余动作Stop the current execution, if any
                    self.arm_moveit.execute(traj,wait=wait_flag)  # 因为上面调用了stop，故理论上无需等待，否则stop将不起作用必然将等待完成；但是不等待机械臂会明显表现出卡顿，因为plan频率过低。
                else: self.cheat_excute()
                self.__update_last_target()
                break
            else:
                if retry_cnt > retry_times: exit('way_points尝试{}次执行失败退出'.format(retry_times))
                retry_cnt+=1
                self.loginfo(f"MoveIt规划失败,fraction={fraction};尝试重新规划。")

        if sleep_time>0: rospy.sleep(sleep_time)
        if info_end != '': print(info_end)

    def quaternion_about_axis(self,angel:float,axis:tuple):
        """ aixs表示要绕着旋转的轴(x,y,z)，angel表示绕轴旋转的角度（deg） """
        return tf_conversions.transformations.quaternion_about_axis(angel,axis)

    def StopMove(self):
        """ 立刻停止运动 """
        cmd_publisher = rospy.Publisher('arx5/joint_command', JointCommand, queue_size=10)
        cmd = JointCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'arx5'
        cmd.mode = 'position'
        cmd.kp = [150 for _ in range(self.joints_num)]
        cmd.kd = [10 for _ in range(self.joints_num)]
        self.get_current_state()
        cmd.position = self.current_joints
        cmd.velocity = [0 for _ in range(self.joints_num)]
        cmd.effort = [50 for _ in range(self.joints_num)]
        cmd.id = [id for id in range(1,self.joints_num+1)]
        self.arm_moveit.stop()  # 停止moveit未必真的管用，比如在不尽兴插值的时候
        for _ in range(20):  # 通过发送之前获取的某个时刻的位置来保证停止
            cmd_publisher.publish(cmd)
            rospy.sleep(0.005)

    def key_get(self,mode:int,timeout=None,print_key=True,pause=False):
        """
           获取键盘输入。模式（mode）设置：
                模式为0时根据timeout来等待获取键值，然后返回键名，函数结束。默认timeout=None一直阻塞等待直到结束;
                模式为1时相当于给模式0在函数中套了个循环，该函数不返回值，而是将值存储起来，通过模式3可以获取该值;
                模式为2时首次调用该函数将启动一个子线程，在子线程中无限循环获取键值，此时timeout始终为none（为防止重复初始化线程，加入了保护机制，只有首次调用该函数时才能启动线程，后续均无效）;
                模式为3时将直接返回已经存储的键值。在无线程情况下意味着返回上次调用模式0或1时的键值，而在有子线程的情况下则意味着返回子线程最近更新的键值。
            pause为true时，利用连续读取两个按键的特点，程序将在首次按下按键后一直暂停，直到另一个按键按键，从而可以实现程序的线程暂停功能。
        """
        if not hasattr(self.key_get,'key'):
            self.key_get.__dict__['key'] = None
            if mode==2:
                Thread(target=self.key_get,daemon=True,args=(1,None)).start()
                return
        while True:
            if mode in [2,3]: return self.key_get.key
            # *****************************************
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                if pause: key = sys.stdin.read(2)  # sys.stdin.read() returns a string on Linux
                else: key = sys.stdin.read(1)
            else: key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            if key == chr(3): exit("Ctrl-C 按下，程序退出")
            # *****************************************
            if print_key: print(f'key={key}')
            self.key_get.__dict__['key'] = key
            if mode==0: return key

    def key_control(self,delta_task=0.01,delta_joint=0.0174,key=None,print_key=False,acc_limit=1.0,vel_limit=1.0,no_plan=False)->str:
        """
            将key转换为控制指令
                delta_task为每次按键移动的工作空间距离值，单位为m/rad
                delta_joint为每次按键移动的关节空间距离值，单位rad
        """
        def limit_and_print():
            if self.key_control.mode == 1:
                max_delta = 0.2
                if self.key_control.__dict__['delta'][1] > max_delta:
                    self.key_control.__dict__['delta'][1] = max_delta
                    print(f'上限保护，joint_delta限制在{max_delta}rad内')
                print(f"当前步长delta_joint={self.key_control.__dict__['delta'][1]}rad")
            else:
                max_delta = 0.1
                if self.key_control.__dict__['delta'][0] > max_delta:
                    self.key_control.__dict__['delta'][0] = max_delta
                    print(f'上限保护，task_delta限制在{max_delta}m/rad内')
                print(f"当前步长delta_task={self.key_control.__dict__['delta'][0]}m/rad")
        if not hasattr(self.key_control,'mode'):
            self.__key_to_target = {'w':1,'a':2,'s':-1,'d':-2,'q':3,'e':-3,'j':-4,'l':4,'i':-5,'k':5,'u':6,'o':-6}
            self.key_control.__dict__['mode'] = 0
            self.key_control.__dict__['delta'] = [delta_task,delta_joint]
            print(f"初始步长为：delta_task={delta_task}m/rad，delta_joint={delta_joint}rad")
        if key is None: key = self.key_get(0,print_key=print_key)
        if key == 't':
            self.key_control.__dict__['mode'] ^= 1
            if self.key_control.__dict__['mode'] == 0:
                print(f"控制空间切换为工作空间，当前步长delta_task={self.key_control.__dict__['delta'][0]}m/rad")
            else: print(f"控制空间切换为关节空间，当前步长delta_joint={self.key_control.__dict__['delta'][1]}rad")
        elif key == 'p':
            if not hasattr(self.gripper_control,'state'):
                self.gripper_control.__dict__['state'] = 'opened'
            if self.gripper_control.__dict__['state'] == 'opened':
                self.gripper_control(1,sleep_time=0,show_state=False)
            else: self.gripper_control(0,sleep_time=0,show_state=False)
        elif key == '[':
            self.key_control.__dict__['delta'][self.key_control.__dict__['mode']]/=10
            limit_and_print()
        elif key == ']':
            self.key_control.__dict__['delta'][self.key_control.__dict__['mode']]*=10
            limit_and_print()
        elif key == 'g':
            self.key_control.__dict__.pop('mode')
            print('g键按下，控制步长恢复默认值')
        elif key == '0':
            self.go_to_named_or_joint_target('Home',sleep_time=1)
        elif self.__key_to_target.get(key) is None:
            return key
        else:
            target = self.__key_to_target[key]  # 借助dict完成了类似C语言的switch-case功能，保证了代码的简洁
            target_abs = abs(target)
            if self.key_control.mode == 0:  # 工作空间控制
                if not no_plan:
                    delta_task = self.key_control.__dict__['delta'][0]
                else: delta_task = 0.003
                delta_task *= target/target_abs  # 正负号控制
                self.go_to_shift_single_axis_target(target_abs-1,delta_task,acc_limit=acc_limit,vel_limit=vel_limit)
            else:  # 关节空间控制
                if not no_plan:
                    delta_joint = self.key_control.__dict__['delta'][1]
                else: delta_joint = 0.0175
                delta_joint *= target/target_abs
                self.go_to_any_joint_target(target_abs-1,delta_joint,acc_limit=acc_limit,vel_limit=vel_limit)
        return key

    def get_current_state(self,sleep_time=0,log_flag=False,get_pose=False):
        """ 得到当前的状态 """
        if sleep_time > 0: rospy.sleep(sleep_time)
        cp = self.arm_moveit.get_current_pose()
        self.current_xyz = [cp.pose.position.x, cp.pose.position.y, cp.pose.position.z]
        self.current_xyzw = [cp.pose.orientation.x,cp.pose.orientation.y,cp.pose.orientation.z,cp.pose.orientation.w]
        self.current_rpy = tf_conversions.transformations.euler_from_quaternion(self.current_xyzw)
        self.current_joints = self.arm_moveit.get_current_joint_values()
        if log_flag:
            self.loginfo("当前状态为: x:{:.2f} y:{:.2f} z:{:.2f} roll:{:.3f} pitch:{:.3f} yaw:{:.3f} (mm/rad)".format(
                                                                                                                self.current_xyz[0]*1000, self.current_xyz[1]*1000, self.current_xyz[2]*1000,
                                                                                                                self.current_rpy[0], self.current_rpy[1], self.current_rpy[2]))
        if get_pose: return cp

    def log_arm_info(self,current_pose=False,target_pose=False,current_joint=False,target_joint=False,sleep=0,rot_mode=0,raw_info=False):
        """ 打印机械臂的一些状态信息 """
        if sleep > 0: rospy.sleep(sleep)
        if current_joint or current_pose: self.get_current_state()
        if current_pose:  # 当前姿态
            if raw_info:  # 打印全部原始数据
                self.loginfo("当前姿态: xyz: [{:.4f},{:.4f},{:.4f}] rpy: [{:.3f},{:.3f},{:.3f}] xyzw: [{:.3f},{:.3f},{:.3f},{:.3f}] (m/rad)".format(
                                                                                                                self.current_xyz[0], self.current_xyz[1], self.current_xyz[2],
                                                                                                                self.current_rpy[0], self.current_rpy[1], self.current_rpy[2],
                                                                                                                self.current_xyzw[0], self.current_xyzw[1], self.current_xyzw[2],self.current_xyzw[3]))
            elif rot_mode == 0:  # 关注rpy
                self.loginfo("当前姿态: x:{:.4f} y:{:.4f} z:{:.4f} roll:{:.3f} pitch:{:.3f} yaw:{:.3f} (m/rad)".format(
                                                                                                                self.current_xyz[0], self.current_xyz[1], self.current_xyz[2],
                                                                                                                self.current_rpy[0], self.current_rpy[1], self.current_rpy[2]))
            elif rot_mode == 1:  # 关注xyzw
                self.loginfo("当前姿态: x:{:.4f} y:{:.4f} z:{:.4f} rot_x:{:.3f} rot_y:{:.3f} rot_z:{:.3f} rot_w:{:.3f} (m/rad)".format(
                                                                                                                self.current_xyz[0], self.current_xyz[1], self.current_xyz[2],
                                                                                                                self.current_xyzw[0], self.current_xyzw[1], self.current_xyzw[2],self.current_xyzw[3]))                
            elif rot_mode == 2: # 都关注
                self.loginfo("当前姿态: x:{:.4f} y:{:.4f} z:{:.4f} roll:{:.3f} pitch:{:.3f} yaw:{:.3f} rot_x:{:.3f} rot_y:{:.3f} rot_z:{:.3f} rot_w:{:.3f} (m/rad)".format(
                                                                                                                self.current_xyz[0], self.current_xyz[1], self.current_xyz[2],
                                                                                                                self.current_rpy[0], self.current_rpy[1], self.current_rpy[2],
                                                                                                                self.current_xyzw[0], self.current_xyzw[1], self.current_xyzw[2],self.current_xyzw[3]))
        if target_pose:  # 目标姿态
            if raw_info:  # 打印全部原始数据
                self.loginfo("目标姿态: xyz: [{:.4f},{:.4f},{:.4f}] rpy: [{:.3f},{:.3f},{:.3f}] xyzw: [{:.3f},{:.3f},{:.3f},{:.3f}] (m/rad)".format(
                                                                                                                self.last_xyz[0], self.last_xyz[1], self.last_xyz[2],
                                                                                                                self.last_rpy[0], self.last_rpy[1], self.last_rpy[2],
                                                                                                                self.last_xyzw[0], self.last_xyzw[1], self.last_xyzw[2],self.last_xyzw[3]))            
            elif rot_mode == 0:
                self.loginfo('目标姿态：x:{:.4f} y:{:.4f} z:{:.4f} roll:{:.3f} pitch:{:.3f} yaw:{:.3f} (m/rad)'.format(self.last_xyz[0],self.last_xyz[1],self.last_xyz[2],
                                                                                                                self.last_rpy[0],self.last_rpy[1],self.last_rpy[2]))
            elif rot_mode == 1:
                self.loginfo('目标姿态：x:{:.4f} y:{:.4f} z:{:.4f} rot_x:{:.3f} rot_y:{:.3f} rot_z:{:.3f} rot_w:{:.3f} (m/rad)'.format(self.last_xyz[0],self.last_xyz[1],self.last_xyz[2],
                                                                                                                self.last_xyzw[0],self.last_xyzw[1],self.last_xyzw[2],self.last_xyzw[3]))
            elif rot_mode == 2:
                self.loginfo('目标姿态：x:{:.4f} y:{:.4f} z:{:.4f} roll:{:.3f} pitch:{:.3f} yaw:{:.3f} rot_x:{:.3f} rot_y:{:.3f} rot_z:{:.3f} rot_w:{:.3f} (m/rad)'.format(self.last_xyz[0],self.last_xyz[1],self.last_xyz[2],
                                                                                                                self.last_rpy[0],self.last_rpy[1],self.last_rpy[2],
                                                                                                                self.last_xyzw[0],self.last_xyzw[1],self.last_xyzw[2],self.last_xyzw[3]))
        if current_joint:
            self.loginfo('当前关节值：{}rad'.format(self.current_joints))
        if target_joint:
            self.loginfo('目标关节值：{}rad'.format(self.last_joint_target))

    def gripper_control(self,pp,sleep_time=0,speed=500,force=100,always=False,show_state=True):
        """
            因时夹爪常用控制接口
            1抓取，0释放；sleep_time是执行动作后的延时
            speed是从1 - 1000 的无量纲速度系数
            force是从50 - 1000 单位为g(克)
            always为True表示持续施加给定的夹取力
        """
        if not self.use_sim:
            def dec_to_hex_string(dec_num:int):
                hex_str = hex(dec_num)[2:].zfill(4).upper()  # 转为16进制，高位补0，转为大写
                byte1 = hex_str[2:]  # 取高8位
                byte2 = hex_str[:2]  # 取低8位
                return byte1 + ' ' + byte2
            def hex_string_sum(hex_string:str,only_low=True):
                """ 输入含16进制元素的字符串，返回各元素相加后的8位16进制数，高位自动补0 """
                hex_list = hex_string.split()  # 将字符串按空格分割成列表
                hex_sum = 0
                for hex_num in hex_list:
                    hex_sum += int(hex_num, 16)  # 将每个16进制数转换成10进制数并相加
                if only_low: hex_sum = hex_sum & 0xFF
                return hex(hex_sum)[2:].zfill(2).upper()
        VID_PID = '6790_29987'  # 因时夹爪的vid和pid（至少目前买的批次是这样）
        if pp == 1:
            self.gripper_control.__dict__['state'] = 'closed'
            if show_state: self.loginfo("夹爪闭合")
            if self.use_sim:
                rospy.set_param("/pick_place_flag","pick")
            else:
                if speed > 1000: speed = 1000
                elif speed < 1: speed = 1
                if force > 1000: force = 1000
                elif force < 50: force = 50
                speed_str = dec_to_hex_string(speed)
                force_str = dec_to_hex_string(force)
                if not always:
                    cmd_temp = '01 05 10 '+speed_str+' '+force_str
                    SUM = hex_string_sum(cmd_temp)
                    CLOSE_CMD = 'EB 90 '+cmd_temp+' '+SUM
                else:
                    cmd_temp = '01 05 18 '+speed_str+' '+force_str
                    SUM = hex_string_sum(cmd_temp)
                    CLOSE_CMD = 'EB 90 '+cmd_temp+' '+SUM
                self.ToSerial(CLOSE_CMD,VID_PID,from_hex=True)
        else:
            self.gripper_control.__dict__['state'] = 'opened'
            if show_state: self.loginfo("夹爪张开")
            if self.use_sim: rospy.set_param("/pick_place_flag","place")
            else:
                speed_str = dec_to_hex_string(speed)
                cmd_temp = '01 03 11 ' + speed_str
                SUM = hex_string_sum(cmd_temp)
                OPEN_CMD = 'EB 90 '+cmd_temp+' '+SUM
                self.ToSerial(OPEN_CMD,VID_PID,from_hex=True)
        rospy.sleep(sleep_time)  # 提示：根据因时的相关协议说明，两次控制指令发送时间间隔最好至少5ms

    def is_alive(self)->bool:
        """ 判断是否关闭了agent """
        if self.node_name is not None:
            if rospy.is_shutdown():
                return False
            else: return True
        else: return True

    def living(self,period=0.005):
        """ 阻塞防止程序退出 """
        if self.node_name is not None: rospy.spin()
        else:
            while True: time.sleep(period)

    def life_end_or_restart(self,mode=0,info=''):
        """ 结束当前智能体的生命周期。0直接结束；1重启 """
        if mode==0:
            if info == '': info = '智能体生命周期结束'
            self.go_to_named_or_joint_target('Home',1)
            exit(info)  # 通过该函数退出当前进程
        else:
            self.restart_times = 0
            for i, arg in enumerate(sys.argv):
                if arg == "--times":
                    self.restart_times = sys.argv[i+1]  # 如果找到了 "--times" 参数，尝试获取它的下一个参数作为值
                    break
            if info == '': info = '智能体重启'
            self.restart_times += 1
            print(info,f'当前重启次数为{self.restart_times}')
            os.execl(sys.executable,sys.executable,*(sys.argv+['--times',f'{self.restart_times}']))  # 通过sys.executable获取当前进程，然后重新执行    

    def ros_spin_once(self,sleep_time=0.5):
        """ 该函数改版自rospy.spin，实现了python中ros的单次spin的spinonce函数 """
        if not hasattr(self.ros_spin_once,'first'):
            self.ros_spin_once.__dict__['first'] = False
            if not rospy.core.is_initialized():
                raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
            rospy.core.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
        try:
            if not rospy.core.is_shutdown(): rospy.rostime.wallsleep(sleep_time)
            else: exit('\r\nROS Core Shut Down')
        except KeyboardInterrupt:
            rospy.core.logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')

    # 在这里维护最新的serial函数
    def ToSerial(self,data_cmd,port_vpid:str,baudrate=115200,bytesize=8,parity='N',stopbits=1,timeout=None,find=False,from_hex=False):
        """
            通过串口发送有关数据；port_pid可以是端口号（Linux下比如是'/dev/ttyUSB0',Windows下比如是'COM8'），也可以是设备的vid和pid信息结合。
            如vid为6790，PID为29987，则写为'6790_29987'。
        """
        if not hasattr(self.ToSerial,f'{port_vpid}'):
            from serial import Serial  # 要装pyserial
            import serial.tools.list_ports
            import atexit
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
                    port = device_list[pid_list.index(int(pid))]
                else: exit('错误：pid和vid与现连接所有设备不匹配')
            ser = MySerial(port,baudrate,bytesize,parity,stopbits,timeout)
            # ser.Connect()
            self.ToSerial.__dict__[f'{port_vpid}'] = ser
            atexit.register(ser.close)  # 程序退出时关闭串口
        # 发送数据
        if data_cmd is not None:
            if from_hex: data_cmd = bytes.fromhex(data_cmd)
            self.ToSerial.__dict__[f'{port_vpid}'].write(data_cmd)

    def ToCAN(self,data,id,bitrate,channel='can0'):
        """ 通过CAN发送消息 """
        if not hasattr(self.ToCAN,channel):
            import can
            # 创建CAN总线对象
            bus = can.interface.Bus(bustype='socketcan', channel=channel, bitrate=bitrate)
            can_msg = can.Message()
            self.ToCAN.__dict__[channel] = (bus,can_msg)  # 每个channel对应一个消息列表，从而限制了同个channel只能单线程发送
        # 消息发送
        self.ToCAN.__dict__[channel][1].arbitration_id = id
        self.ToCAN.__dict__[channel][1].data = data
        self.ToCAN.__dict__[channel][0].send(self.ToCAN.__dict__[channel][1])
        # # 接收CAN消息
        # while True:
        #     can_msg = bus.recv()
        #     if can_msg is not None:
        #         print(f"Received CAN message: ID={can_msg.arbitration_id}, data={can_msg.data}")
        #         break
        return

    def loginfo(self,info='',sleep_f=0,sleep_b=0):
        """ 可带节点名的消息打印 """
        if sleep_f >0: rospy.sleep(sleep_f)
        if self.node_name is None: print(info)
        else: rospy.loginfo("[{}] {}".format(self.node_name, info))
        if sleep_b >0: rospy.sleep(sleep_b)

    def logwarn(self,warn=''):
        """ 可带节点名的警告打印 """
        if self.node_name is None: print('WARN:',warn)
        else: rospy.logwarn("[{}] {}".format(self.node_name, warn))

    """ 以上是通用接口 """
    """ **************************************************************************************************************** """
    """ 以下是pick&place任务的接口 """

    def task_pick_place_param_init(self,area_mode=0,use_tof=False):
        """ 仅涉及x、y方向覆盖调整，不涉及z """
        self.use_tof = use_tof
        # 滤波有关变量
        self.average_count = 0
        self.pose_buf = np.zeros((7,20))
        if not self.use_sim:  # 实机
            self.wait_near_judge = 0.0005  # 多近关闭wait,一般取0.5mm比较好
            # 滤波相关变量
            self.filter_times = 1  # 滤波次数
            self.filter_near_judge = 10  # 多近开启滤波
            if area_mode == 0:
                self._place_xy = (-0.27,0.01)  # 放置时的基准位置（实机y为0容易出现卡位，因此稍微在y向上有一些偏移，不影响叠放逻辑）
                self._place_rpy = (self._pick_rpy[0],self._pick_rpy[1],math.pi) # place的y与pick时的y相差180度    
                # 监测高度时的x位置，以保证测距传感器能测到物块高度为宜 # TODO
                if use_tof: self._place_detect_x = self._place_xy[0] + 0.01
                else: self._place_detect_x = self._place_xy[0]
            else:
                self._place_xy = (0.303,0.204)
                self._place_rpy = self._pick_rpy
                if use_tof: self._place_detect_x = self._place_xy[0] - 0.01
                else: self._place_detect_x = self._place_xy[0]
        else:  # 仿真
            self.wait_near_judge = 0.0005  # 多近关闭wait,一般取0.5mm比较好;0为不关闭
            # 滤波相关变量
            self.filter_times = 1  # 仿真色块识别不滤波
            self.filter_near_judge = 10
            self._place_detect_x = self._place_xy[0]  # 仿真和不使用detect时都跟self._place_xy[0]保持一致
        self.pick_place_area_mode = area_mode
        """ 初始化PickPlace任务相关参数，以及控制机械臂到初始位置 """
        self._pick_place_stage = 'pick'  # 首次进入始终意味着阶段初始化为pick
        self.pick_place_0_1 = 0
        self.__auto_pick_place = False
        self._cube_height = rospy.get_param("~cube_height", default=0.025)
        """ 有些参数仿真和实机可以兼容（通常是相对移动的距离参数），有些需要分别配置（一般是绝对姿态参数） """
        self._gap_disget = 0.003  # 检测物块搭建的实际高度时的位置，应较大
        self._gap_place  = 0.003  # 视觉调整时两个物块之间的上下距离间隙，应较小
        # 高度限制
        self._max_z = 0.253  # 机械臂z轴支持的最高高度，实测大概0.26左右，取0.25
        self._max_cubes_num = int((self._max_z-self._place_base_z)/self._cube_height) + 1  # 计算得到的理论上支持的最多的物块数量
        self.loginfo(f'支持叠放的物块数量最多为：{self._max_cubes_num}')

    def AutoFollow(self):
        """ 开始自动跟随任务 """
        if not hasattr(self.AutoFollow,'first'):
            self.AutoFollow.__dict__['first'] = False
            self.go_to_pick_pose(True,2)
            # 订阅视觉反馈
            rospy.Subscriber("/target_TF",TransformStamped,self.feedback_callback,queue_size=1)  # queue_size=1表明只接收最新数据
            self.loginfo("Subscribe to /target_TF to receive vision feedback.")
            # 启动跟踪线程
            self._follow_event = Event()  # 跟踪线程的event
            Thread(target=self.__follow,daemon=True).start()
            self.loginfo('Follow Tread Started.')
            # 使能视觉反馈
            rospy.set_param("/vision_attention",'pick'+str(self.pick_place_0_1))
        else: print('请勿重复调用start_auto_follow函数')

    def PickPlace(self,pick_keyboard=False,place_mode=0,start_base=0,use_gripper=True):
        """
            pick_keyboard为真时pick和place均通过键盘控制完成，为假则pick自动完成，place根据mode配置。
            place_mode为0时，采用自动闭环控制；为1时采用自动半闭环控制；为2时采用键盘控制。
            start_base表示初始化时认为已经叠放好的物块个数，默认为0。
        """
        self.__pick_keyboard = pick_keyboard
        self.__start_base = start_base
        self._first_place_satisfy = False
        self._first_pick_satisfy = False
        self._first_out = 0
        self._change_pick_place_state.__dict__['times'] = self.__start_base  # times参数可以用来记录叠放次数，同时也可用于确定place的高度，而其奇偶决定待抓取颜色（该参数实际记录的是从place切到pick的次数）
        self.pick_place_0_1 = self._change_pick_place_state.times % 2
        if start_base >= self._max_cubes_num:
            exit(f"初始值{start_base}应小于上限值{self._max_cubes_num}")
        else: print(f"初始叠放物块数：{start_base}")
        if self.__pick_keyboard:
            rospy.set_param("/vision_attention",'pause')  # 关闭视觉反馈
            if place_mode != 2:
                self.place_mode = 2
                print('pick为键控时place也需为键控，已自动调整')
        else: self.place_mode = place_mode
        """ 开始自动叠放任务 """
        self.__auto_pick_place = True
        # 进入初始化位置
        if use_gripper: self.gripper_control(0,sleep_time=0)  # 确保夹爪开启
        self.AutoFollow()
        Thread(target=self.__pick_place_test,daemon=True,args=(pick_keyboard,place_mode,0.5)).start()

    def _max_deviation(self)->float:
        """ 获得当前6D数据中的最大值，用于进行near_judge """
        self.max_dis = 0.0
        for i in range(3):
            if self.max_dis < abs(self.feedback_target_position[i]):
                self.max_dis = abs(self.feedback_target_position[i])
            if self.max_dis < abs(self.feedback_target_euler[i])/10:
                self.max_dis = abs(self.feedback_target_euler[i])/10
        return self.max_dis # 返回归一化的6D的归一化最大距离

    def _near_judge(self,start_max=0.01)->bool:
        """ start_max以xyz的距离为标准，0.01表示1cm的范畴，对应角度为0.1rad，即5.72度 """
        if self.max_dis < start_max:
            return True
        else:
            return False

    def _change_feedback_target_to_list(self,relative_target:TransformStamped):
        """ 获得反馈的偏差数据并进行适当转换处理 """
        self.get_current_state()  # 得到当前状态
        self.feedback_target_position = [relative_target.transform.translation.x,relative_target.transform.translation.y,relative_target.transform.translation.z]
        self.feedback_target_euler = list(tf_conversions.transformations.euler_from_quaternion([relative_target.transform.rotation.x,relative_target.transform.rotation.y,
                                                                relative_target.transform.rotation.z,relative_target.transform.rotation.w]))
        # 跟踪色块时(且是pick阶段)根据 此时的yaw偏角（应注意，世界坐标系下，此时末端关节的转动不再是roll而是yaw） 对trans进行额外处理，转换xy相对世界坐标系为相对夹爪朝向本身
        if self.__auto_pick_place and self._pick_place_stage == 'pick':
            # 求出yaw的偏差
            current_yaw_bias = -(self._pick_rpy[2] - self.current_rpy[2])  # 以顺时针偏转时该bias为正值代入后续计算
            # 根据当前的roll偏差解算出实际应该移动的前后和左右（z轴和y轴）距离（近似认为当前roll和反馈的数据是同步的）
            tf_trans_x =  self.feedback_target_position[0]*math.cos(current_yaw_bias) - self.feedback_target_position[1]*math.sin(current_yaw_bias)  # 左右移动
            self.feedback_target_position[1] = self.feedback_target_position[0]*math.sin(current_yaw_bias) + self.feedback_target_position[1]*math.cos(current_yaw_bias)  # 前后移动
            self.feedback_target_position[0] = tf_trans_x  # 临时保存的用完

    def _feedback_moving_smooth(self,nums=1,near_dis=0.001)->bool:
        """ 滑动/移动均值滤波法 """
        self._max_deviation()  # 执行一次这个函数以获得当前的最大偏差
        if self._near_judge(near_dis):
            if(nums>1):
                for i in range(3):
                    self.pose_buf[i][self.average_count] = self.feedback_target_position[i]
                    self.pose_buf[i+3][self.average_count] = self.feedback_target_euler[i]
                self.average_count+=1
                if(self.average_count==nums):
                    self.average_count = 0
                    self.pose_buf[6][0] = 7
                if(self.pose_buf[6][0] == 7):
                    for i in range(3):
                        self.feedback_target_position[i]  = sum(self.pose_buf[i])/nums
                    for i in range(3):
                        self.feedback_target_euler[i] = sum(self.pose_buf[i+3])/nums
                    return True  # 滤波完成
                return False     # 滤波未完成
        else:
            self.pose_buf[6][0] = 0
            self.average_count=0
        # loginfo("距离较远或num=1,无需滤波！\r\n")
        return True

    def _feedback_average_smooth(self,nums,near_dis=0.001)->bool:
        """ 均值滤波:在对相机帧率要求不高，甚至需要降低帧率的时候，使用均值滤波以达到更稳定的效果 """
        self._max_deviation()  # 执行一次这个函数以获得当前的最大偏差
        if not hasattr(self._feedback_average_smooth,'times'):
            self._feedback_average_smooth.__dict__['times']=0
            self.__sum = np.zeros((6,nums))
        if self._near_judge(near_dis):
            if nums > 1:
                times = self._feedback_average_smooth.__dict__['times']
                self.__sum[:,times] = np.array(self.feedback_target_position+self.feedback_target_euler)
                self._feedback_average_smooth.__dict__['times'] += 1
                if self._feedback_average_smooth.__dict__['times'] == nums:
                    sum_:np.ndarray = np.sum(self.__sum,axis=1)/nums
                    temp = sum_.tolist()
                    self.feedback_target_position, self.feedback_target_euler = temp[:3],temp[3:6]
                    self._feedback_average_smooth.__dict__['times'] = 0
                    return True
        else: self._feedback_average_smooth.__dict__['times']=0;self.__sum = np.zeros((6,nums))
        return False

    def _set_const_bias_cmd_target(self,bias_xyz=0.0001,neardis_xyz=0.001,min_xyz=0.0002,bias_rpy=0.1,neardis_rpy=1,min_rpy=0.15):
        """
        通过定值的重复执行某个小步长以积分方式慢慢逐步逼近,const_target远距离时设定的值应为30ms左右移动距离，
        而由于一般远距离同时允许连续执行，因此，该值设的稍微大点也无所谓。近距离时，应根据精度要求设定较小的值。
        """
        self._set_const_bias_xyz_target(bias_xyz,neardis_xyz,min_xyz)
        self._set_const_bias_rpy_target(bias_rpy,neardis_rpy,min_rpy)

    def _set_const_bias_xyz_target(self,bias_xyz=0.0001,neardis_xyz=0.001,min_xyz=0.0002):
        abs_trans = [abs(self.feedback_target_position[i]) for i in range(3)]
        for i in range(3):
            if abs_trans[i] <= neardis_xyz:
                if abs_trans[i] > min_xyz:
                    if self.feedback_target_position[i]>0:
                        self.new_target_xyz[i] =  bias_xyz
                    else:
                        self.new_target_xyz[i] = -bias_xyz
                else:
                    self.new_target_xyz[i] = 0

    def _set_const_bias_rpy_target(self,bias_rpy=0.1,neardis_rpy=1,min_rpy=0.15):
        """ 这里rpy输入参数的单位均是deg，方便人的直观配置"""
        min_rpy *= 0.01745
        neardis_rpy *= 0.01745
        bias_rpy *= 0.01745
        abs_angles = [abs(self.feedback_target_euler[i]) for i in range(3)]
        for i in range(3):
            if abs_angles[i] <= neardis_rpy:
                if abs_angles[i] > bias_rpy:  # 0.4度
                    if self.feedback_target_euler[i]>0:
                        self.new_target_rpy[i] =  bias_rpy  # 以0.15度为增量
                    else:
                        self.new_target_rpy[i] = -bias_rpy
                else:
                     self.new_target_rpy[i] = 0

    def _set_pixels2meter_xyz_target(self,k,mindis=4):
        """ 当距离超过mindis时，直接设置xyz目标为像素对应的距离值，从而更快地到达目标点，并且分段代码也将更加简洁 """
        abs_trans = [abs(self.feedback_target_position[i]) for i in range(3)]
        for i in range(3):
            if abs_trans[i] > mindis:
                self.new_target_xyz[i] = self.feedback_target_position[i]/k

    def _feedback_target_dichotomia(self,near_dis=0.0003,dichotomia=2.0,test_log=None):
        """ 偏差N分（默认二分，即Kp=0.5） """
        if self._near_judge(near_dis):
            for i in range(3):
                self.new_target_xyz[i] = self.feedback_target_position[i]/dichotomia
                self.new_target_rpy[i] = self.feedback_target_euler[i]/dichotomia
            if test_log is not None:
                self.loginfo(test_log)
            return True
        return False

    def _feedback_target_pid(self):
        """ 利用视觉偏差进行pid控制 """
        return

    def _clear_meaningless_deviation(self,min_t_error=1,min_p_error=2,precision_whole=1)->bool:  
        """
        按实际可达精度或要求精度对无效的小数位进行四舍五入处理（precision为1代表整体偏差在0.1以下时忽略，以此类推）
        # min_t_error=1,min_p_error=2一般不用再改了，这就是精度的固定水平了
        """
        if precision_whole > 0:
            self.error_abs_sum = 0.0  # 6D偏差的绝对值之和，反映了总体的偏差。单位：m
            for i in range(3):
                self.feedback_target_position[i]  = round(self.feedback_target_position[i],min_t_error+3) # 0.1mm精度，0.0001
                self.feedback_target_euler[i] = round(self.feedback_target_euler[i],min_p_error)  # 0.01rad精度，即0.572度
                self.error_abs_sum += (abs(self.feedback_target_position[i]) + abs(self.feedback_target_euler[i]))
            # 判断清洗后是否所有值均为0（为避免浮点问题，同一个很小的数进行比较）
            if self.error_abs_sum*(10**(min_t_error+3)) < precision_whole:  # 如precision=1,若求和为0.0001m，即0.1mm，则结果为1，返回为True
                self.loginfo("距离目标较近,放弃MoveIt控制!\r\n")
                return False
            return True
        else:  # 精度为0表示不需要进行clear
            self.error_abs_sum = 112233
            return True

    def _auto_vel_acc_limit(self,min_vel_factor=1.0,min_acc_factor=0.1):
        """ 自动速度、加速度限制（系数对应可能得好好调一下） """
        limit_factor =  self._max_deviation()
        if limit_factor > 1:
            limit_factor=1.0
        elif limit_factor < min_acc_factor:
            limit_factor = min_acc_factor
        self.arm_moveit.set_max_acceleration_scaling_factor(limit_factor) # 加速度限制
        if limit_factor < min_vel_factor:
            limit_factor = min_vel_factor
        self.arm_moveit.set_max_velocity_scaling_factor(limit_factor)  # 速度限制

    def _manual_vel_acc_limit(self,near_dist=0.005,vel_limit=1.0,acc_limit=0.01):
        """ 手动指定在特定距离下的速度、加速度限制(注意顺序是自上而下、由大至小) """
        if self._near_judge(near_dist):
            self.arm_moveit.set_max_acceleration_scaling_factor(acc_limit) # 加速度限制
            self.arm_moveit.set_max_velocity_scaling_factor(vel_limit)  # 速度限制

    def _set_link_vel_acc_maxval(self,vel_max):
        """ 限制link运动的速度（注意是link，而不是joint；无加速度限制接口） """
        for i in range(1,6):
            self.arm_moveit.limit_max_cartesian_link_speed(vel_max, link_name="link{}".format(i))
        self.arm_moveit.limit_max_cartesian_link_speed(vel_max, link_name="gripper_link1")
        self.arm_moveit.limit_max_cartesian_link_speed(vel_max, link_name="gripper_link2")

    def _motion_optimization(self,always_wait=True):
        """ 运动控制优化 """
        if not self.use_sim:  # 实机
            # if not always_wait and self._near_judge(self.wait_near_judge):  # 近距离0.5mm时
            #     self.wait_flag=False  # 近距离控制无需等待
            # else: self.wait_flag=True  # 等待执行完
            # # 自动线性限速（还是手动更方便精细些）
            # # self.auto_vel_acc_limit(min_vel_factor=1.0,min_acc_factor=0.07)  # 加速度和速度根据偏差大小自动调整，类似于一个额外的位置环
            # 手动多段限速
            self._manual_vel_acc_limit(near_dist=640,vel_limit=0.407,acc_limit=1.0)
            self._manual_vel_acc_limit(near_dist=10, vel_limit=0.2,acc_limit=0.5)
            self._manual_vel_acc_limit(near_dist=5,vel_limit=0.1,acc_limit=0.1)
        else:  # 仿真
            self._manual_vel_acc_limit(near_dist=640,vel_limit=1.0,acc_limit=1.0)
            self._manual_vel_acc_limit(near_dist=320,vel_limit=0.9,acc_limit=0.5)
            self._manual_vel_acc_limit(near_dist=160,vel_limit=0.8,acc_limit=0.2)
            self._manual_vel_acc_limit(near_dist=80, vel_limit=0.5,acc_limit=0.1)
            self._manual_vel_acc_limit(near_dist=20, vel_limit=0.2,acc_limit=0.01)
            self._manual_vel_acc_limit(near_dist=10, vel_limit=0.1,acc_limit=0.001)

    def feedback_callback(self,target_message:TransformStamped):  
        if not hasattr(self.feedback_callback,'running'):
            self.feedback_callback.__dict__['running'] = False
        # 避免函数被重复运行（ROS多个话题发布时，某个订阅者的回调函数将会被开启多次，即便上次的回调函数还没执行完）
        if self.feedback_callback.__dict__['running']: return
        else: self.feedback_callback.__dict__['running'] = True
        self.feedback_callback.__dict__['time'] = time.time()  # 获得当前时间(正常有数据情况下刷新频率约为33ms)，用于执行情况检测
        # 计算获取当前与目标的相对距离
        self._change_feedback_target_to_list(target_message)
        # ******视觉反馈的偏差数据滤波********（原始数据30Hz，33ms）
        if self._feedback_moving_smooth(self.filter_times,self.filter_near_judge) and not self._follow_event.is_set():
            # 当前的最大偏差
            self._max_deviation()
            # 调试信息打印
            if self.use_sim: self.__log_pose_info(feedback=True,enable=True)
            else: self.__log_pose_info(feedback=True,absum=True,enable=False)
            # 判断并执行
            self.judge_and_excute()
        # 声明函数退出
        self.feedback_callback.__dict__['running'] = False

    def judge_and_excute(self):
        if self.__auto_pick_place:
            if self._pick_place_stage == 'pick':
                self._Pick_and_Go(stable=True,pick_keyboard=self.__pick_keyboard,place_mode=self.place_mode)
            if self._pick_place_stage == 'place':  # 不用elif，可以直接当次进入
                self._Place_and_Go(stable=True,pick_keyboard=self.__pick_keyboard,place_mode=self.place_mode)
        else: self._follow_event.set()  # 仅跟踪

    def set_control_param(self,mode=0):
        """ 课程用 """
        self.set_control_param.__dict__['first'] = mode
        self.__jiumin = mode
        rospy.set_param('control_param',math.inf)

    def convert_feedback_to_target(self):
        # 课程用参数动态调整（初始化为正无穷大，则此时理论上机械臂不会进行任何移动）
        if hasattr(self.set_control_param,'first'):
            pik = rospy.get_param('control_param1',default=math.inf)*100
            plc = rospy.get_param('control_param2',default=math.inf)*100
        else: pik = plc = 0

        # 纯像素新版参数调整（将反馈数据转换为self.new_target_xyz和rpy）
        if self.use_sim:
            if self._pick_place_stage == 'pick':
                if pik != 0:
                    self._set_pixels2meter_xyz_target(k=pik,mindis=0)  # k建议直接实测得到，并根据实际情况进行适当的调整
                else:
                    self._set_pixels2meter_xyz_target(k=5555,mindis=0)  # 粗调。仿真中k建议直接实测得到，并根据实际情况进行适当的调整
                    self._set_const_bias_xyz_target(bias_xyz=0.002, neardis_xyz=20, min_xyz=2)
                    self._set_const_bias_xyz_target(bias_xyz=0.0005,neardis_xyz=10, min_xyz=2)
                    self._set_const_bias_xyz_target(bias_xyz=0.0002,neardis_xyz=5, min_xyz=2)
            else:
                if plc != 0:
                    self._set_pixels2meter_xyz_target(k=plc,mindis=0)  # k建议直接实测得到，并根据实际情况进行适当的调整
                else:
                    self._set_const_bias_cmd_target(bias_xyz=0.015, neardis_xyz=640,min_xyz=2,bias_rpy=10,  neardis_rpy=180,min_rpy=0.2)
                    self._set_const_bias_cmd_target(bias_xyz=0.004, neardis_xyz=100,min_xyz=2,bias_rpy=5,neardis_rpy=15,min_rpy=0.2)
                    self._set_const_bias_cmd_target(bias_xyz=0.002, neardis_xyz=50,min_xyz=2,bias_rpy=2,neardis_rpy=5,min_rpy=0.2)
                    self._set_const_bias_cmd_target(bias_xyz=0.0013, neardis_xyz=20, min_xyz=2, bias_rpy=1,neardis_rpy=2,min_rpy=0.2)
                    self._set_const_bias_cmd_target(bias_xyz=0.0008,neardis_xyz=10, min_xyz=2,bias_rpy=0.5,neardis_rpy=1,min_rpy=0.2)
                    self._set_const_bias_cmd_target(bias_xyz=0.0004,neardis_xyz=5, min_xyz=2,bias_rpy=0.2,neardis_rpy=0.5,min_rpy=0.2)

            self.new_target_rpy = self.feedback_target_euler  # rpy（实际只有y有偏差值）直接一次到目标
        else:  # 实机参数
            if self._pick_place_stage == 'pick':
                if plc == 0:
                    self._set_pixels2meter_xyz_target(k=6000,mindis=20)  # 粗调。仿真中k建议直接实测得到，并根据实际情况进行适当的调整
                    self._set_const_bias_xyz_target(bias_xyz=0.002, neardis_xyz=20, min_xyz=2)
                    self._set_const_bias_xyz_target(bias_xyz=0.0005,neardis_xyz=10, min_xyz=2)
                    self._set_const_bias_xyz_target(bias_xyz=0.0002,neardis_xyz=5, min_xyz=2)
                else:
                    self._set_pixels2meter_xyz_target(k=pik,mindis=0)
            else:
                if plc != 0:
                    self._set_pixels2meter_xyz_target(k=plc,mindis=0)  # k建议直接实测得到，并根据实际情况进行适当的调整
                else:
                    self._set_const_bias_xyz_target(bias_xyz=0.002, neardis_xyz=640, min_xyz=2)
                    self._set_const_bias_xyz_target(bias_xyz=0.001, neardis_xyz=20, min_xyz=2)
                    self._set_const_bias_xyz_target(bias_xyz=0.00025,neardis_xyz=10, min_xyz=2)
                    self._set_const_bias_xyz_target(bias_xyz=0.0001,neardis_xyz=5, min_xyz=2)
            self.new_target_rpy = self.feedback_target_euler  # rpy（实际只有y有偏差值）直接一次到目标
        self.__set_given_target_pose(self.new_target_xyz,self.new_target_rpy,target_base='last')

    def __follow(self):
        if not hasattr(self.__follow,'thread'):
            self.__follow.__dict__['thread'] = True
            self.follow_wait = True  # 这个根据自己的情况灵活更改
        """ 该函数应在单独线程中运行 """
        while True:
            if self.__follow.thread == False: break  # 退出线程
            # 等待执行
            self._follow_event.wait()
            # 反馈值转换为目标值
            self.convert_feedback_to_target()
            # 优化控制与执行
            self._motion_optimization()
            succeed_flag = self.__go_to_pose_target(self.start_base_mode,self.follow_wait,self.precise_pose,return_enable=True,retry_times=0)
            # if succeed_flag: self.loginfo(f'执行完毕，本次目标xyz:{self.target_pose_position}')
            # clear，从而可以刷新pose
            self._follow_event.clear()

    def _Pick_and_Go(self,start_xy=5,start_yaw=0.0175*1,stable=False,pick_keyboard=False,place_mode=0):
        """
            半闭环抓起物块然后移动到待放置的位置的，但不立刻开环放置
            # TODO：目前仍然有一小段开环下降抓取过程，后续可以通过两段参考的方式实现全闭环
        """
        if place_mode: set_vision = False
        else: set_vision = True
        if pick_keyboard:
            print("可以开始手动调节夹爪位置，调节好后按'g'键进入下一阶段")
            while self.key_control(delta_task=0.01) != 'g':pass
            self.gripper_control(1,sleep_time=1)  # 然后发送抓取指令
            self._change_pick_place_state('place',place_mode=not(place_mode),set_vision=set_vision)  # 切换状态
            return
        if self._first_out != 0:
            self._first_out -= 1
        elif (abs(self.feedback_target_position[0])+abs(self.feedback_target_position[1]) < start_xy) and self.feedback_target_euler[2] < start_yaw:  # 当xyroll偏差小于一定像素值时，向下到可以抓取物体的高度
            if stable and not self._first_pick_satisfy:
                self._first_pick_satisfy = True
                rospy.sleep(0.5)  # 冷静，待机械臂稳定
                return
            rospy.set_param("/vision_attention",'pause')  # 关闭视觉反馈
            self.joints_speed_control(max_=0.15)  # 限制一下速度减少抖动
            self.go_to_single_axis_target(2,self._pick_base_z,sleep_time=1)  # 首先到达可抓取的高度位置(z单轴移动)
            self.gripper_control(1,sleep_time=1)  # 然后发送抓取指令
            self._change_pick_place_state('place',place_mode=place_mode,set_vision=set_vision)  # 切换状态
            self._first_pick_satisfy = False
        else:
            if self._first_pick_satisfy:
                self._first_pick_satisfy = False
            self._follow_event.set()

    def _Place_and_Go(self,x_bias=2,y_bias=2,stable=False,pick_keyboard=False,place_mode=0):
        """ 达到指定位置后，开始进行视觉闭环调节。条件满足后立刻放置物块 """
        if place_mode == 2:  # 键盘控制
            print("可以开始手动调节夹爪位置，调节好后按'g'键进入下一阶段")
            while self.key_control(delta_task=0.001) != 'g':pass
            self.gripper_control(0,sleep_time=0.5)  # 然后发送释放指令
            self._change_pick_place_state('pick',set_vision=not(pick_keyboard))  # 切换状态
        elif place_mode == 1:  # 开环控制
            self.go_to_shift_single_axis_target(2,-self._gap_place,sleep_time=1)  # 下降到恰好相接触处进行释放
            self.gripper_control(0,sleep_time=0.5)  # 然后发送释放指令
            self._change_pick_place_state('pick',set_vision=not(pick_keyboard))  # 状态切换到pick
        # 闭环控制
        elif (self._change_pick_place_state.times == 0) or (abs(self.feedback_target_position[0]) < x_bias and abs(self.feedback_target_position[1]) < y_bias and self._first_out == 0):
            if self._change_pick_place_state.times == 0: self.loginfo("放置首个物块")  # 首次Place
            else:
                if self.__jiumin == 1:
                    self._first_place_satisfy = True
                    self._follow_event.set()  # 条件不满足，继续调节
                    return
                if stable and not self._first_place_satisfy:
                    self._first_place_satisfy = True
                    rospy.sleep(0.5)  # 冷静，待机械臂稳定
                    return
            rospy.set_param("/vision_attention",'pause')  # 关闭视觉反馈
            # 下降到恰好相接触处进行释放
            if self._change_pick_place_state.times != 0: self.go_to_shift_single_axis_target(2,-self._gap_place,sleep_time=1.5)
            self.gripper_control(0,sleep_time=0.5)
            # 状态切换到pick
            self._change_pick_place_state('pick',set_vision=not(pick_keyboard))
            self._first_place_satisfy = False
        # 每一轮中首次从pick出去并且进入place，并且不是第一轮进入，那么需要刷新偏差值（为保险，刷新3遍）再重新进行判断，故在这里进行回退
        elif self._first_out != 0:
            self._first_out -= 1
            # self.loginfo(f"pick后首次place，刷新3遍视觉反馈：{3-self._first_out}")
        else:
            self._first_place_satisfy = False
            self._follow_event.set()  # 条件不满足，继续调节

    def _change_pick_place_state(self,pick_or_place:str,place_mode=True,set_vision=True):
        """ 每个阶段以夹爪的闭合/开启为结束，然后进行状态的change """
        # 内部状态信息切换
        self._pick_place_stage = pick_or_place
        # 执行动作
        if pick_or_place=='place':  # 进入place位置以备视觉修正
            if self._change_pick_place_state.__dict__['times'] == 0 and place_mode != 2:
                self.go_to_place_pose(first=True)  # 第一次特殊，不经过高度测量以及调节，直接到达待放置位置
            else: self.go_to_place_pose(use_tof=self.use_tof,place_mode=place_mode)
        elif pick_or_place=='pick':  # 进入pick位置开始新一轮pp
            self._change_pick_place_state.__dict__['times'] += 1  # 每次change到pick，意味着新一轮开始了，此时将次数进行累加
            self._place_base_z = self._pick_base_z + self._cube_height * self._change_pick_place_state.times
            self.pick_place_0_1 = self._change_pick_place_state.times % 2 # 刷新01状态
            self.go_to_pick_pose(sleep_time=2,from_place=True,use_name=True)  # 移动到pick区域
            # 如果下次的z轴已经超过最高点，则直接结束（#TODO：后续统一放在debug后的self.monitor_and_change_pick_region()函数中）
            if self._place_base_z + self._gap_disget > self._max_z:
                self.life_end_or_restart(0,info='已达到支持的最高堆叠高度，程序自动退出')
            # # 开启无反馈检测线程
            # self.monitor_and_change_pick_region()
        self._first_out = 3  # 刚出去标志
        if set_vision: rospy.set_param("/vision_attention",self._pick_place_stage+str(self.pick_place_0_1))  # 对2取余判断奇偶，0和1

    def go_to_pick_pose(self,use_name=False,sleep_time=1,from_place=False):
        """ 到达pick位姿 """
        if from_place:
            if self.pick_place_area_mode == 0:
                pose1 = deepcopy(self.last_target_pose.pose)
                pose1.position.z = self.last_xyz[2] + self._cube_height + 0.01  # 先上升
                pose2 = deepcopy(pose1)
                pose2.position.y = self.last_xyz[1] + self._cube_height + 0.01  # 然后向右移动一段距离
                self.set_and_go_to_way_points([pose1,pose2])
            else:
                pose1 = deepcopy(self.last_target_pose.pose)
                pose1.position.z = self.last_xyz[2] + self._cube_height + 0.01  # 先上升
                pose2 = deepcopy(pose1)
                pose2.position.y = self.last_xyz[1] - 0.075  # 然后向右移动一段距离
                self.set_and_go_to_way_points([pose1,pose2])

        if use_name:
            if self.pick_place_area_mode == 0:
                if self.use_sim: self.go_to_named_or_joint_target('PickSim')
                else: self.go_to_named_or_joint_target("PickUnder")
            else:
                if self.use_sim: self.go_to_named_or_joint_target('PickSim')
                else: self.go_to_named_or_joint_target("PickUnder")
            vel_limit = acc_limit = 0.1
        else: vel_limit = acc_limit = 1.0

        # 精确的工作空间移动
        if self.pick_place_area_mode == 0:
            suc = self.set_and_go_to_pose_target(self._pick_scan_xyz_dict[self.pick_place_0_1],self._pick_rpy,sleep_time=sleep_time,return_enable=True)
            if not suc and not use_name:  # 多次规划失败则尝试先进行named移动
                if self.use_sim: self.go_to_named_or_joint_target("PickSim")
                else: self.go_to_named_or_joint_target("PickUnder")
                self.set_and_go_to_pose_target(self._pick_scan_xyz_dict[self.pick_place_0_1],self._pick_rpy,sleep_time=sleep_time)
            elif not suc: exit('执行pick姿态失败')
        else:
            suc = self.set_and_go_to_pose_target(self._pick_scan_xyz_dict[self.pick_place_0_1],self._pick_rpy,sleep_time=sleep_time,return_enable=True,vel_limit=vel_limit,acc_limit=acc_limit)
            if not suc and not use_name:  # 多次规划失败则尝试先进行named移动
                if self.use_sim: self.go_to_named_or_joint_target("PickSim")
                else: self.go_to_named_or_joint_target("PickUnder")
                self.set_and_go_to_pose_target(self._pick_scan_xyz_dict[self.pick_place_0_1],self._pick_rpy,sleep_time=sleep_time)
            elif not suc: exit('执行pick姿态失败')

    def go_to_place_pose(self,first=False,use_tof=True,place_mode=0):
        """ 多种方式到达place位姿 """
        def prepare(first):  # pick区域先上升，然后调整xy为place时的位置
            pose1 = deepcopy(self.last_target_pose.pose)
            if first: pose1.position.z = self.last_xyz[2] + self._cube_height + 0.01
            else: pose1.position.z = disget_z
            pose2 = deepcopy(pose1)
            if self.pick_place_area_mode != 1:  # 反向一波
                pose2.position.x,pose2.position.y = -self._place_xy[0],-self._place_xy[1]
            elif self.pick_place_area_mode == 1:
                pose2.position.x,pose2.position.y = self._place_xy[0],self._place_xy[1]
            self.set_and_go_to_way_points([pose1,pose2],vel_limit=0.407,acc_limit=1)
        if first:
            prepare(first)
            if self.pick_place_area_mode != 1: self.go_to_named_or_joint_target({0:3})  # 中间段保证轨迹单向性
            if self.use_sim:
                self.set_and_go_to_pose_target([*self._place_xy,self._place_base_z+self._gap_place],self._place_rpy,sleep_time=1)
            else:  # 实机由于各种误差因素需要更细致的控制
                # self.speed_control(0.4,0.5)
                self.set_and_go_to_pose_target([*self._place_xy,self._pick_base_z+self._cube_height],self._place_rpy,sleep_time=0.5,vel_limit=0.4,acc_limit=0.2)  # 先保证xy对正，放置卡位造成xy无法到达目标引起偏差
                # self.speed_control(0.4,0.5)
                self.set_and_go_to_pose_target([*self._place_xy,self._place_base_z],self._place_rpy,sleep_time=1,vel_limit=0.2,acc_limit=0.2)
            return
        else:
            disget_z = self._place_base_z + self._gap_disget
            if place_mode == 2: disget_z += 0.025  # 键控时要留够余量
            # 首先在pick的区域抬高到进行高度测量时的高度（这样可以避免移动时碰到其它物块）
            prepare(first)
            # 然后移动到place区域顶层物块的高度测量处（以全过程不碰到顶层物块为宜;由于实机转向的限位，因此分成两段，保证转向始终沿俯视的逆时针方向，这也符合大多数机械臂的实际转动情况）
            if self.pick_place_area_mode != 1: self.go_to_named_or_joint_target({0:3})  # 中间段保证轨迹单向性
            if place_mode == 2:
                self.set_and_go_to_pose_target([*self._place_xy,disget_z],self._place_rpy)
                return
            if use_tof:
                self.set_and_go_to_pose_target([self._place_detect_x,self._place_xy[1],disget_z],self._place_rpy,sleep_time=2,vel_limit=0.4,acc_limit=0.2)
            else: self.set_and_go_to_pose_target([*self._place_xy,disget_z],self._place_rpy,sleep_time=1,vel_limit=0.4,acc_limit=0.2)
            # 停稳后（一定要保证停稳），获取物块此时的配合值（必然大于0且小于(0.025+self._above_gap_disget)，若大于0.025则上次物块搭建是失败了的，为此整轮应进行回退）
            self.distance_detect(use_sim=self.use_sim,use_tof=use_tof)
            # 然后移动到可以开始调节的位置
            adjust_z = disget_z-self.delta_cube_z+self._gap_place
            if self.use_sim:
                self.set_and_go_to_pose_target([*self._place_xy,adjust_z],self._place_rpy,sleep_time=2)
            else:  # 实机当高度增加时，会出现x方向前移的问题，造成下层物块难识别，为此调节位置x方向向后偏移一定距离
                if place_mode == 0:  # 闭环自动（实机经过一些特殊处理）
                    self.set_and_go_to_pose_target([self._place_xy[0],self._place_xy[1],adjust_z],self._place_rpy,sleep_time=2)
                    self.loginfo(f'到达调节位置，开始闭环叠放第{self._change_pick_place_state.times+1}个物块')
                    # exit('测试，程序退出')
                elif place_mode == 1:  # 开环自动
                    self.set_and_go_to_pose_target([self._place_xy[0],self._place_xy[1],adjust_z],self._place_rpy,sleep_time=2)

    def pick_place_success_judge(self)->bool:
        """ # TODO：检查是否成功抓放，若检测到抓放失败则本轮刷新 """
        return True

    def monitor_and_change_pick_region(self,__in_thread=False):
        """
            当一定次数连续没有检测到目标时，认为当前位置已经无目标，移动到新的检测位置
            首次执行时创建线程，并且当前线程未结束时，不会创建新的线程，直到结束后再次执行该函数才会创建新的线程
            并且该函数只有在其自己创建的线程中执行相应功能，其它不执行（也就是说__in_thread参数不要修改其默认值）
        """
        if not hasattr(self.monitor_and_change_pick_region,'times'):
            self.monitor_and_change_pick_region.__dict__['times'] = {0:[0,0],1:[0,0],2:False}  # key 0和1表示self.pick_place_0_1状态，元素两个值分别表示机械臂x轴和y轴的change次数
            self.monitor_and_change_pick_region.__dict__['thread'] = False
            self._pick_region_delta = 0.025
            self.cornor_region = [1,2,self._pick_scan_xyz_dict[0][2]]  # TODO:待确定边角位置（一般右下角，因为x和y增加是朝左上角移动的，具备一致性）
            self.__pick_scan_xyz_dict = deepcopy(self._pick_scan_xyz_dict)

        # 如果下次的z轴已经超过最高点，则直接结束
        if self._place_base_z+self._gap_disget > self._max_z:
            self.life_end_or_restart(0,'已达到支持的最高堆叠高度，程序自动退出')

        # 判断是否有未完成线程
        if self.monitor_and_change_pick_region.thread == False:
            self.monitor_and_change_pick_region.__dict__['thread'] = True
            feedback_time = self.feedback_callback.time
            Thread(target = self.monitor_and_change_pick_region,daemon=True,args=(True,)).start()  # 启动检测线程
        else: self.loginfo('有尚未完成的线程,本次不设置线程')

        if __in_thread:  # 线程中才会执行该部分代码
            self.loginfo('启动可夹取物块存在性监察线程')
            monitor_times=0
            sleep = 0.5  # 检测频率
            time_out = 3  # 容许时间
            func_times:list = self.monitor_and_change_pick_region.times[self.pick_place_0_1]  # 取出当前状态的列表（由于可变对象是引用赋值，所以取出后的改动将连锁到原变量）
            pick_scan_xyz = self._pick_scan_xyz_dict[self.pick_place_0_1]  # 取出当前状态
            base_axis = 1
            another_axis = base_axis^1  # ^不是幂运算，而是异或运算，可以实现0和1的不断求反切换功能
            max_convert_xy = [4,4]  # xy方向总共支持的改变的次数
            while True:  # 线程中循环执行
                rospy.sleep(sleep)
                if self.feedback_callback.time != feedback_time:  # 若发生更新，则清空计数
                    monitor_times = 0
                    feedback_time = self.feedback_callback.time
                else: monitor_times+=1  # 否则计数累加
                # 若夹爪状态为关闭，则表明已经夹到物块了，可以不必检测了，退出线程
                if self.gripper_control.state == 'closed': break
                # 若超时，则表明需要更换位置了
                elif monitor_times*sleep > time_out:
                    # 若任何一个阶段的参考方向到达终点，则回到初始位置，并提醒添加物块，物块添加好后，键盘按下某个键，可以重新开始！
                    if self.monitor_and_change_pick_region.times[2]:
                        print('已无可继续识别的相应颜色的物块，本次搭建结束，回到起点，可重新添加物块后按6键继续进行搭建，按0键重新进行搭建，或者按.键退出程序。')
                        self._pick_scan_xyz_dict = deepcopy(self.__pick_scan_xyz_dict)  # 恢复最初的起点位置
                        self.set_and_go_to_pose_target(pick_scan_xyz,self._pick_rpy,sleep_time=2)
                        while True:
                            key = self.key_get()
                            if key == '6':
                                print('开始继续搭建')
                                return
                            elif key == '0':  # 重新搭建，最简单的就是直接重启一遍程序，但实际上程序难以自动重启，因此采用软重启：清空之前的所有ros内容，然后删除实例变量，然后生成一个新的实例变量
                                self.life_end_or_restart(1,'开始重新搭建')
                                return  # 此时退出后，应该没有任何与该实例相关活动程序了
                            elif key == '.':
                                self.life_end_or_restart(0,'搭建完成进程结束')
                                return
                            else: print('请按正确的按键：6、0或.')
                    self.loginfo('长时间未检测到目标，更换检测区域。')
                    # 首次更换区域，设置右下角作为后续依次change的起点位置  # TODO:后续直接初始位置应该就是边缘位置，或者将整个pick区域通过代号划分成几个部分，然后用户可以通过代号进行选择初始化位置，而后续程序也可以根据代号选择后续调整时跳过该初始化位置的重复检测。
                    if func_times[base_axis] == 0:
                        func_times[base_axis] = 1
                        pick_scan_xyz = self.cornor_region
                    else:  # 后续开始按S型累加移动
                        func_times[base_axis] += 1
                        if func_times[base_axis] > max_convert_xy[base_axis]:  # 到达y方向的pick区域的边界处了，x增加，y不变
                            pick_scan_xyz[another_axis] += self._pick_region_delta
                            func_times[another_axis] += 1  # 另一个轴计数累加
                            func_times[base_axis] = 1  # 重新归1
                            if func_times[another_axis] >= max_convert_xy[base_axis]:
                                self.monitor_and_change_pick_region.times[2] = True
                        else:
                            if func_times[another_axis]%2 == 0:
                                pick_scan_xyz[base_axis] += self._pick_region_delta
                            else: pick_scan_xyz[base_axis] -= self._pick_region_delta
                    # 到达新的位置
                    self.set_and_go_to_pose_target(pick_scan_xyz,self._pick_rpy,sleep_time=2)
                    self._first_out = 2
                    self.loginfo(f'目前共更换了{func_times}次')
                    monitor_times = 0  # 计数清零，继续检测和更换区域，直到成功为止
            # 退出线程前，将thread标志设为false，便于下次再次启用检测
            self.monitor_and_change_pick_region.__dict__['thread'] = False
            self.loginfo('监察完毕，退出可夹取物块存在性监察线程')
        else: self.loginfo("非独立线程执行，本次执行无效")

    def distance_detect(self,use_sim=False,use_tof=False,sleep_time=0):
        """ 距离测量 """
        if sleep_time > 0: rospy.sleep(sleep_time)
        if use_sim:
            self.delta_cube_z = rospy.get_param('/cube_delta_z')  # 获得最高物块与次高物块的高度差
            self.loginfo('开始测量到顶端物块的实际距离')
        else:
            if use_tof:
                self.loginfo('开始测量到顶端物块的实际距离')
                tof2cubebottom = 0.01  # 测量得到的tof至夹爪夹取的物块的下底面的距离，可通过简介测量，即测量tof到已放置的物块的距离，减去用卡尺测量到的两物块间隙。
                dis = rospy.get_param('/distance')
                self.delta_cube_z = dis - tof2cubebottom
            else: self.delta_cube_z = self._gap_disget  # 越小越不容易碰到之前的物块- 0.001
        # 有效性判断
        if self.delta_cube_z < 0 or self.delta_cube_z > (self._cube_height+self._gap_disget):
            raise Exception(f'间隙值异常,叠放失败结束。间隙：{self.delta_cube_z}，超出理论边界[0,{0.025+self._gap_disget}]')
        elif use_tof: self.loginfo(f'得到两物块间隙高度为：{self.delta_cube_z:.3f}'+'m'+'，开始进行高度调整')

    def __pick_place_test(self,pick=True,place=True,sleep_time=0.5):
        """通过自行发送0target数据进行过程模拟"""
        target_pub = rospy.Publisher("/target_TF",TransformStamped,queue_size=1) # queue_size=1表明只发布最新数据
        target = TransformStamped()
        target.header.stamp = rospy.Time.now()
        target.transform.translation.x = 0
        target.transform.translation.y = 0
        target.transform.translation.z = 0
        tf_q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
        target.transform.rotation.x = tf_q[0]
        target.transform.rotation.y = tf_q[1]
        target.transform.rotation.z = tf_q[2]
        target.transform.rotation.w = tf_q[3]
        while (pick or place):
            if pick and self._pick_place_stage == 'pick':
                target_pub.publish(target)
            if place and self._pick_place_stage == 'place':
                target_pub.publish(target)
            rospy.sleep(sleep_time)

    def __log_pose_info(self,feedback=False,max_err=False,current=False,absum=False,target=False,intergral=False,extra_front='',extra_end='',enable=True):
        """ 打印一些信息 """
        if enable:
            if feedback is True:  # 反馈量
                self.loginfo(extra_front+"反馈量为: x:{:.1f} y:{:.1f} z:{:.1f} roll:{:.1f} pitch:{:.1f} yaw:{:.3f} (pixel/rad)".format(
                                                                                                                self.feedback_target_position[0] ,self.feedback_target_position[1],self.feedback_target_position[2],
                                                                                                                self.feedback_target_euler[0],self.feedback_target_euler[1],self.feedback_target_euler[2])+extra_end)                     
      
            if absum is True:  # 绝对目标位置
                self.loginfo(extra_front+"总体偏差为：{}(绝对值之和,m/rad)".format(self.error_abs_sum)+extra_end)
            if max_err is True:  # 反馈的最大偏差
                self.loginfo(extra_front+"偏差最大为：{}  (m/rad)".format(self.max_dis)+extra_end)
            if current is True:  # 机械臂当前姿态
                self.get_current_state()
                self.loginfo(extra_front+"当前状态为: x:{:.2f} y:{:.2f} z:{:.2f} roll:{:.3f} pitch:{:.3f} yaw:{:.3f} (mm/rad)".format(
                                                                                                                    self.current_xyz[0]*1000, self.current_xyz[1]*1000, self.current_xyz[2]*1000,
                                                                                                                    self.current_rpy[0], self.current_rpy[1], self.current_rpy[2])+extra_end)
            if target is True:  # 目标值
                self.loginfo(extra_front+'最新目标为：x:{:.3f}mm y:{:.3f}mm z:{:.3f}mm'.format(self.new_target_xyz[0]*1000,self.new_target_xyz[1]*1000,self.new_target_xyz[2]*1000)+extra_end)