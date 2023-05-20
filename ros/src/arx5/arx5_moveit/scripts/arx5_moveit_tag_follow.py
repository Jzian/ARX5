#!/usr/bin/env python3

from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy
import tf_conversions
import numpy as np
import threading

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

NODE_NAME = 'arx5_moveit_tag_follow'

def loginfo(msg: str):
    rospy.loginfo("[{}] {}".format(NODE_NAME, msg))

def logwarn(msg: str):
    rospy.logwarn("[{}] {}".format(NODE_NAME, msg))

class MoveitTagFollowNode(object):

    def __init__(self) -> None:

        rospy.init_node(NODE_NAME)
        loginfo("Initializing {} Node.".format(NODE_NAME))
        # 选择是键盘控制0,还是自动跟随二维码1、还是rviz控制3，还是键盘和自动同时启用4
        self.follow_mode = rospy.get_param("~FollowMode", default=1)
        # 选择是仿真还是实机（有不同的参数和控制方式）
        self.use_sim = rospy.get_param("~use_sim", default=True)
        # 相对目标偏差的处理方式
        self.use_integral = True   # T则采用偏差积分的手段得到目标偏差数据
        # 两个基准的确定标志
        self.target_base_mode = 1  # 即target设定的基准是基于当前状态0还是基于初始化时的固定状态1，前者必须要保证偏差量要大于重力影响，而后者的偏差量应进行积分，否则控制将出现问题。# 基于目标的控制方式，如果下次的目标设定时上次的动作未执行完，将导致设定的起始点和当前状态点不再仅仅是差重力偏差，久而久之将造成无法规划。
        self.start_base_mode  = 1  # 即moveit轨迹规划的起点位置是当前状态0还是上次目标1
        # 执行的一些控制标志
        self.wait_mode = 1  # 1表示一直wait，0表示一直不wait，2表示视情况进行wait
        # 初始化参数
        self.param_init()
        # 初始化moveit
        self.moveit_init()
        # 初始化模式
        self.mode_set('gravity_integral')
        # 根据不同模式订阅不同的控制指令
        if self.follow_mode==1:  # 自动跟随二维码
            self.target_sub = rospy.Subscriber("/target_TF",TransformStamped,self.target_callback,queue_size=1)
            loginfo("Subscribe to /target_TF, MODE: {}\r\n".format(self.follow_mode))
        elif self.follow_mode==0:  # 键盘控制
            self.use_integral = False
            self.target_base_mode = 0
            self.start_base_mode  = 0
            self.target_sub = rospy.Subscriber("/arx5/moveit_cmd",TransformStamped,self.target_callback,queue_size=1)
            self.filter_times = 1  # 滤波设置为1，保证及时相应键盘命令
            self.discrimination = 0  # 不必控制精度了
            loginfo("Subscribe to /arx5/moveit_cmd\r\n")

    # 许多mode是有待测试的(mode决定了目标数据处理和moveit控制的不同方式)
    def mode_set(self,mode:str):
        if mode == 'traditional':  # 实机可用
            self.use_integral = False   # 是否采用偏差积分的手段
            self.target_base_mode = 0  # 即target设定的基准是基于当前状态还是基于上次目标，前者必须要保证偏差量要大于重力影响
            self.start_base_mode  = 0  # 即moveit轨迹规划的起点位置是当前状态还是上次目标
        elif mode == 'no-gravity_integral':  # 实机可用
            self.use_integral = True
            self.target_base_mode = 0
            self.start_base_mode  = 0
        elif mode == 'gravity_integral':  # SIM可用
            self.use_integral = True
            self.target_base_mode = 1
            self.start_base_mode  = 1
        elif mode == 'local_integral':
            self.use_integral = True
            self.target_base_mode = 1
            self.start_base_mode  = 0
        elif mode == 'constant_exe':
            self.use_integral = True
            self.target_base_mode = 1
            self.start_base_mode  = 1
            self.wait_flag = True
        # self.target_base_mode=0， self.start_base_mode=1这种情况是不合理的

    # 根据是实机还是仿真进行参数的不同初始化
    def param_init(self):
        self.new_target_xyz   = [0,0,0]
        self.new_target_rpy   = [0,0,0]
        # 积分有关变量
        self.average_count = 0
        self.pose_buf = np.zeros((7,20))
        self.integral_sum_xyz = [0,0,0]
        self.integral_sum_rpy = [0,0,0]
        # 同步控制
        self.new_feedback = False
        # 实机
        if not self.use_sim:
            # 与moveit控制精度、稳定性有关的变量（总体误差1mm，xyz控制精度至少应达到0.1mm，其分辨力至多为0.033mm）
            self.discrimination:int = 3  # 鉴别力阈，单位：0.1mm（适当增加可以提高稳定性，从而达到综合的较小误差;经实验，设置在0.3-0.5mm较为不错，否则稳定性误差可能将达到将近1mm；0表示不设置阈值）
            self.precise_pose = True  # 表示给定的pose是精确的值
            self.position_tolerance = 0.00005  # 0.05mm的目标的位置公差
            self.orientation_tolerance = 0.01  # 0.01rad的目标的角度公差
            self.joint_tolerance = 0.00017 # 0.01°的目标关节角度公差
            self.wait_near_judge = 0.0005  # 多近关闭wait,一般取0.5mm比较好
            # 滤波相关变量
            self.filter_times = 18  # 滤波次数
            self.filter_near_judge = 0.001  # 多近开启滤波
        # 仿真
        else:
            self.discrimination:int = 0
            self.precise_pose = True # 表示给定的pose是精确的值(公差由下面的变量决定，表现为一直发同样的目标moveit)
            self.position_tolerance = 0.000005
            self.orientation_tolerance = 0.001
            self.joint_tolerance = 0.000017
            self.wait_near_judge = 0.0005  # 多近关闭wait,一般取0.5mm比较好;0为不关闭
            # 滤波相关变量
            self.filter_times = 18  # 滤波次数
            self.filter_near_judge = 0.001  # 多近开启滤波

    # 初始化moveit接口
    def moveit_init(self):
        rospy.sleep(4.5)  # 略等一会儿，防止同一个launch文件启动时，moveit接口还没启动好，导致报错，4.5s延时可以
        self.reference_frame = 'base_link'
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.reference_frame
        self.arm_moveit = MoveGroupCommander('arx5_arm')
        if not self.arm_moveit.has_end_effector_link():
            raise Exception("ERROR: 未找到末端link,请检查配置\r\n")
        else:
            self.end_effector_link = self.arm_moveit.get_end_effector_link()
        self.arm_moveit.set_pose_reference_frame(self.reference_frame)
        # plan规划配置
        self.arm_moveit.allow_replanning(False)  # 快速跟踪时这里通常设置为false防止卡住。依靠实时的更新数据重新规划。
        self.arm_moveit.set_planning_time(1)  # 经验证，可以为浮点数
        self.arm_moveit.set_num_planning_attempts(1) # 规划的尝试次数
        # 目标精度控制
        self.arm_moveit.set_goal_position_tolerance(self.position_tolerance)  # 设定位置误差允许值,m;0.001为1mm的误差之内
        self.arm_moveit.set_goal_orientation_tolerance(self.orientation_tolerance)  # 设定角度误差允许值,rad
        self.arm_moveit.set_goal_joint_tolerance(self.joint_tolerance)  # 即0.01度
        # 初始化为最大的速度和加速度
        self.arm_moveit.set_max_acceleration_scaling_factor(1.0) # 加速度限制
        self.arm_moveit.set_max_velocity_scaling_factor(1.0)     # 速度限制
        # 进入待识别准备位置
        self.go_to_init_target()

    # 到初始位置，并保存此时的初始状态
    def go_to_init_target(self):
        if self.follow_mode==1:
            self.go_to_named_target("Standby")
        elif self.follow_mode==2 or self.follow_mode==0:
            self.go_to_named_target("PickReady")
        # 获取当前初始状态(尽管与目标存在一定偏差，但仅限之后的首次执行有影响，后续均不再有影响)
        init_pose = self.arm_moveit.get_current_pose()
        self.init_xyz = [init_pose.pose.position.x, init_pose.pose.position.y, init_pose.pose.position.z]
        init_pitch, init_roll, init_yaw = self.arm_moveit.get_current_rpy()
        self.init_rpy = [init_roll, init_pitch, init_yaw]
        return

    # 到指定名称位置，并更新上次目标角度
    def go_to_named_target(self,target_name):
        self.arm_moveit.set_named_target(target_name)
        # 得到这一次的目标关节角度
        self.last_joint_target = self.arm_moveit.get_joint_value_target()  # 获取joint目标状态，从而某些情况下可以用来作为下一次规划的起始状态
        # 执行
        self.arm_moveit.go(wait=True)
        self.arm_moveit.stop() # Calling `stop()` ensures that there is no residual movement(避免多余动作)
        loginfo("{} Pose Over\r\n".format(target_name))

    # 获得反馈的偏差数据并进行适当转换处理
    def get_feedback_target(self,relative_target:TransformStamped):
        self.tf_trans = [relative_target.transform.translation.x,relative_target.transform.translation.y,relative_target.transform.translation.z]
        self.tf_angles = tf_conversions.transformations.euler_from_quaternion([relative_target.transform.rotation.x,relative_target.transform.rotation.y,
                                                                relative_target.transform.rotation.z,relative_target.transform.rotation.w])
        self.tf_angles = list(self.tf_angles)
        return self.tf_trans,self.tf_angles

    # 得到当前的状态
    def get_current_state(self):
        cp = self.arm_moveit.get_current_pose()
        self.current_xyz = [cp.pose.position.x, cp.pose.position.y, cp.pose.position.z]
        cpitch, croll, cyaw = self.arm_moveit.get_current_rpy()
        self.current_rpy = [croll, cpitch, cyaw]

    # 打印一些信息
    def pose_info_log(self,rela=False,max_err=False,nowtime=False,absum=False):
        if rela is True:
            loginfo("6D偏差为: x:{:.2f} y:{:.2f} z:{:.2f} roll:{:.3f} pitch:{:.3f} yaw:{:.3f} (mm/rad)\r\n".format(
                                                                                                                self.tf_trans[0]*1000 ,self.tf_trans[1]*1000,self.tf_trans[2]*1000,
                                                                                                                self.tf_angles[0],self.tf_angles[1],self.tf_angles[2]))          
        if absum is True:
            loginfo("总体偏差为：{}(绝对值之和,m/rad)\r\n".format(self.error_abs_sum))
        if max_err is True:
            loginfo("偏差最大为：{}  (m/rad)\r\n".format(self.max_dis))
        if nowtime is True:
            self.get_current_state()  # 保险起见再获取一遍最新的状态
            loginfo("当前状态为: x:{:.2f} y:{:.2f} z:{:.2f} roll:{:.3f} pitch:{:.3f} yaw:{:.3f} (mm/rad)\r\n".format(
                                                                                                                self.current_xyz[0]*1000, self.current_xyz[1]*1000, self.current_xyz[2]*1000,
                                                                                                                self.current_rpy[0], self.current_rpy[1], self.current_rpy[2]))

    # 获得当前6D数据中的最大值，用于进行near—judge
    def max_deviation(self)->float:
        self.max_dis = 0.0
        for i in range(3):
            if self.max_dis < abs(self.tf_trans[i]):
                self.max_dis = abs(self.tf_trans[i])
            if self.max_dis < abs(self.tf_angles[i])/10:
                self.max_dis = abs(self.tf_angles[i])/10
        return self.max_dis # 返回归一化的6D的归一化最大距离

    # start_max以xyz的距离为标准，0.01表示1cm的范畴，对应角度为0.1rad，即5.72度
    def near_judge(self,start_max=0.01)->bool:
        if self.max_dis < start_max:
            return True
        else:
            return False

    # 偏差值滑动平均滤波函数
    def FeedbackTargetSmooth(self,nums=1,near_dis=0.001)->bool:
        self.max_deviation()  # 执行一次这个函数以获得当前的最大偏差
        if self.near_judge(near_dis):
            if(nums>1):
                for i in range(3):
                    self.pose_buf[i][self.average_count] = self.tf_trans[i]
                    self.pose_buf[i+3][self.average_count] = self.tf_angles[i]
                self.average_count+=1
                if(self.average_count==nums):
                    self.average_count = 0
                    self.pose_buf[6][0] = 7
                if(self.pose_buf[6][0] == 7):
                    self.tf_trans[0]  = sum(self.pose_buf[0])/nums
                    self.tf_trans[1]  = sum(self.pose_buf[1])/nums
                    self.tf_trans[2]  = sum(self.pose_buf[2])/nums
                    self.tf_angles[0] = sum(self.pose_buf[3])/nums
                    self.tf_angles[1] = sum(self.pose_buf[4])/nums
                    self.tf_angles[2] = sum(self.pose_buf[5])/nums
                    return True  # 滤波完成
                return False     # 滤波未完成
        else:
            self.pose_buf[6][0] = 0
            self.average_count=0
        # loginfo("距离较远或num=1,无需滤波！\r\n")
        return True

    # 通过定值的重复执行某个小步长以积分方式慢慢逐步逼近,const_target远距离时设定的值应为30ms左右移动距离，而由于一般远距离同时允许连续执行，因此，该值设的稍微大点也无所谓。近距离时，应根据精度要求设定较小的值。
    def SetConstBiasCmdTarget(self,bias_xyz=0.0001,neardis_xyz=0.001,min_xyz=0.0002,bias_rpy=0.1,neardis_rpy=1,min_rpy=0.15):
        """ 这里rpy输入参数的单位均是deg"""
        min_rpy *= 0.01745
        neardis_rpy *= 0.01745
        bias_rpy *= 0.01745
        abs_trans = [abs(self.tf_trans[i]) for i in range(3)]
        abs_angles = [abs(self.tf_angles[i]) for i in range(3)]
        for i in range(3):
            # xyz
            if abs_trans[i] <= neardis_xyz:
                if abs_trans[i] > min_xyz:
                    if self.tf_trans[i]>0:
                        self.new_target_xyz[i] =  bias_xyz
                    else:
                        self.new_target_xyz[i] = -bias_xyz
                else:
                     self.new_target_xyz[i] = 0
            # rpy
            if abs_angles[i] <= neardis_rpy:
                if abs_angles[i] > bias_rpy:  # 0.4度
                    if self.tf_angles[i]>0:
                        self.new_target_rpy[i] =  bias_rpy  # 以0.15度为增量
                    else:
                        self.new_target_rpy[i] = -bias_rpy
                else:
                     self.new_target_rpy[i] = 0

    # 偏差N分（默认二分，即Kp=0.5）
    def FeedbackTargetDichotomia(self,near_dis=0.0003,dichotomia=2.0,test_log=None):
        if self.near_judge(near_dis):
            for i in range(3):
                self.new_target_xyz[i] = self.tf_trans[i]/dichotomia
                self.new_target_rpy[i] = self.tf_angles[i]/dichotomia
            if test_log is not None:
                loginfo(test_log)
            return True
        return False

    # 按实际可达精度或要求精度对无效的小数位进行四舍五入处理（precision为1代表整体偏差在0.1以下时忽略，以此类推）
    def clear_meaningless_deviation(self,min_t_error=1,min_p_error=2,precision_whole=1)->bool:  # min_t_error=1,min_p_error=2一般不用再改了，这就是精度的固定水平了
        if precision_whole > 0:
            self.error_abs_sum = 0.0  # 6D偏差的绝对值之和，反映了总体的偏差。单位：m
            for i in range(3):
                self.tf_trans[i]  = round(self.tf_trans[i],min_t_error+3) # 0.1mm精度，0.0001
                self.tf_angles[i] = round(self.tf_angles[i],min_p_error)  # 0.01rad精度，即0.572度
                self.error_abs_sum += (abs(self.tf_trans[i]) + abs(self.tf_angles[i]))
            # 判断清洗后是否所有值均为0（为避免浮点问题，同一个很小的数进行比较）
            if self.error_abs_sum*(10**(min_t_error+3)) < precision_whole:  # 如precision=1,若求和为0.0001m，即0.1mm，则结果为1，返回为True
                loginfo("距离目标较近,放弃MoveIt控制!\r\n")
                return False
            return True
        else:  # 精度为0表示不需要进行clear
            self.error_abs_sum = 112233
            return True

    # 自动速度、加速度限制（系数对应可能得好好调一下）
    def _auto_vel_acc_limit(self,min_vel_factor=1.0,min_acc_factor=0.1):
        limit_factor =  self.max_deviation()
        if limit_factor > 1:
            limit_factor=1.0
        elif limit_factor < min_acc_factor:
            limit_factor = min_acc_factor
        self.arm_moveit.set_max_acceleration_scaling_factor(limit_factor) # 加速度限制
        if limit_factor < min_vel_factor:
            limit_factor = min_vel_factor
        self.arm_moveit.set_max_velocity_scaling_factor(limit_factor)  # 速度限制

    # 手动指定在特定距离下的速度、加速度限制(注意顺序是自上而下、由大至小)
    def _manual_vel_acc_limit(self,near_dist=0.005,vel_limit=1.0,acc_limit=0.01):
        if self.near_judge(near_dist):
            self.arm_moveit.set_max_acceleration_scaling_factor(acc_limit) # 加速度限制
            self.arm_moveit.set_max_velocity_scaling_factor(vel_limit)  # 速度限制

    # 限制link运动的速度（注意是link，而不是joint；无加速度限制接口）
    def _set_vel_acc_maxval(self,vel_max):
        for i in range(1,6):
            self.arm_moveit.limit_max_cartesian_link_speed(vel_max, link_name="link{}".format(i))
        self.arm_moveit.limit_max_cartesian_link_speed(vel_max, link_name="gripper_link1")
        self.arm_moveit.limit_max_cartesian_link_speed(vel_max, link_name="gripper_link2")

    # 运动控制优化
    def motion_optimization(self):
        if not self.use_sim:
            if self.near_judge(self.wait_near_judge):  # 近距离0.5mm时
                self.wait_flag=False  # 近距离控制无需等待
            else:
                self.wait_flag=True  # 远距离等待执行完
            # 自动线性限速
            # self.auto_vel_acc_limit(min_vel_factor=1.0,min_acc_factor=0.07)  # 加速度和速度根据偏差大小自动调整，类似于一个额外的位置环
            # 或手动多段限速
            self._manual_vel_acc_limit(near_dist=1,     vel_limit=1.0,acc_limit=1.0)
            self._manual_vel_acc_limit(near_dist=0.05,  vel_limit=0.9,acc_limit=0.5)
            self._manual_vel_acc_limit(near_dist=0.005, vel_limit=0.8,acc_limit=0.05)
            self._manual_vel_acc_limit(near_dist=0.0015,vel_limit=0.1,acc_limit=0.005)
            self._manual_vel_acc_limit(near_dist=0.0005,vel_limit=0.1,acc_limit=0.001)
        else:
            # 手动多段限速
            self._manual_vel_acc_limit(near_dist=1,     vel_limit=1.0,acc_limit=1.0)
            self._manual_vel_acc_limit(near_dist=0.05,  vel_limit=0.9,acc_limit=0.5)
            self._manual_vel_acc_limit(near_dist=0.005, vel_limit=0.8,acc_limit=0.05)
            self._manual_vel_acc_limit(near_dist=0.0015,vel_limit=0.1,acc_limit=0.005)
            self._manual_vel_acc_limit(near_dist=0.0005,vel_limit=0.1,acc_limit=0.001)            

    # 设定起始状态为上次的目标cmd
    def set_start_state_to_last_target_cmd(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.arm_moveit.get_active_joints() # active不包含virtual joint ["base_link_to_link1","link1_to_link2","link2_to_link3","link3_to_link4","link4_to_link5","link5_to_gripper_link1"] #  
        joint_state.position = self.last_joint_target  # 初始值在gotonamed中赋值，后续均在设置moveit目标后更新
        # print(joint_state.name , joint_state.position)  # 调试用
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.arm_moveit.set_start_state(moveit_robot_state)

    # 设定目标的基准和相对偏差，得到绝对的最终目标
    def set_target_pose(self,target_base_xyz,target_base_rpy,rela_target_xyz,rela_target_rpy):
        # 计算新的相对目标
        rela_target_wxyz = tf_conversions.transformations.quaternion_from_euler(target_base_rpy[1]+rela_target_rpy[1],target_base_rpy[0]+rela_target_rpy[0],target_base_rpy[2]+rela_target_rpy[2])
        # 得到绝对目标
        self.target_pose.pose.position.x = target_base_xyz[0] + rela_target_xyz[0]
        self.target_pose.pose.position.y = target_base_xyz[1] + rela_target_xyz[1]
        self.target_pose.pose.position.z = target_base_xyz[2] + rela_target_xyz[2]
        self.target_pose.pose.orientation.x = rela_target_wxyz[0]
        self.target_pose.pose.orientation.y = rela_target_wxyz[1]
        self.target_pose.pose.orientation.z = rela_target_wxyz[2]
        self.target_pose.pose.orientation.w = rela_target_wxyz[3]

    # 接收最新的偏差数据进行处理，得到相关信息量
    def target_callback(self,target_message:TransformStamped):
        # 立刻得到当前的状态，以进行相对控制（近似认为该状态与之前得到的trans是同时的）
        self.get_current_state()
        # 计算获取当前与目标的相对距离
        self.get_feedback_target(target_message)
        # ******视觉反馈的偏差数据滤波********（原始数据30Hz，33ms）
        if self.FeedbackTargetSmooth(self.filter_times,self.filter_near_judge) and not self.new_feedback:
            # ********清理无精度意义的偏差数据，保持数据稳定在精度范围内********
            meaning_flag = self.clear_meaningless_deviation(precision_whole=self.discrimination)  # 所有数据均为0，则反馈flag=false(放弃moveit控制），否则为true；同时得到error_abs_sum。
            if not meaning_flag:  # 若数据无意义，则直接返回不执行了
                return
            # 获得清洗后的当前的最大偏差
            self.max_deviation()

            # if self.use_sim:
            #     # # 目标积分(积分自带滤波作用)
            #     # self.TargetIntegral(near_dis=0.001,ki=0.1)
            #     # # 目标二分（偏差较小，但大于分辨力时，采用二分运动）
            #     # self.TargetDichotomia(near_dis=0.0003,dichotomia=2.0)
            #     # 常值积分
            #     self.TargetConstIntegral(const_target=0.001,neardis_single=0.01,min_dis=0.005)
            # else:
            #     pass

            # 调试信息打印
            if self.use_sim:
                self.pose_info_log(rela=True)
            else:
                if self.follow_mode==0:
                    self.pose_info_log(nowtime=True)
                else:
                    self.pose_info_log(rela=True,absum=True)

            """ ********以上均为视觉反馈数据处理部分;以下为cmd控制目标处理部分;二者的处理是独立的,即后者的处理不会影响前者原有的值,因为前者将在moveit执行时作为参考量******** """

            # 积分方式
            if self.use_integral:
                # 偏差N分处理(基于当前最大偏差)
                flag =self.FeedbackTargetDichotomia(near_dis=1,dichotomia=1.5)
                flag&=self.FeedbackTargetDichotomia(near_dis=0.5,dichotomia=5)
                flag&=self.FeedbackTargetDichotomia(near_dis=0.01,dichotomia=10)
                flag&=self.FeedbackTargetDichotomia(near_dis=0.0025,dichotomia=15)
                # 偏差定值处理（基于每个偏差自身的大小）
                if flag:
                    self.SetConstBiasCmdTarget(bias_xyz=0.0001,neardis_xyz=0.001,min_xyz=0.0002,bias_rpy=0.1,neardis_rpy=1,min_rpy=0.2)
                    print("偏差定值处理阶段:",self.new_target_xyz,self.new_target_rpy)
                # 偏差累加
                self.integral_sum_xyz = [self.integral_sum_xyz[i]+self.new_target_xyz[i] for i in range(3)]
                self.integral_sum_rpy = [self.integral_sum_rpy[i]+self.new_target_rpy[i] for i in range(3)]
                # 设定最终目标值
                if self.target_base_mode == 1:
                    self.set_target_pose(self.init_xyz,self.init_rpy, self.integral_sum_xyz,self.integral_sum_rpy)
                else:
                    self.set_target_pose(self.current_xyz,self.current_rpy, self.integral_sum_xyz,self.integral_sum_rpy)
                    logwarn("警告：积分方式时采用current为基准时，其意义在于跳出死区，但前提是重力影响可忽略或积分量大于重力影响，否则可能适得其反。")
            # 非积分方式
            else:
                # 设定最终目标值
                if self.target_base_mode == 1:
                    logwarn("警告：self.target_base_mode == 1，即目标为固定初始状态时，必须采用积分方式，否则将导致偏差失去合理的基准。")
                else:
                    self.set_target_pose(self.current_xyz,self.current_rpy,self.tf_trans,self.tf_angles)

            # 允许进行控制
            self.new_feedback = True

    def MoveitFollow(self):
        self.wait_flag = True  # 动作执行过程是否阻塞
        while True:
            if self.follow_mode==3:  # 纯rviz控制，本程序无需进行控制
                rospy.sleep(0.002)
            elif self.new_feedback:
                self.arm_moveit.stop() # 避免多余动作
                # ****优化控制****
                self.motion_optimization()
                self.arm_moveit.clear_pose_targets()  # It is always good to clear your targets after planning with poses
                # loginfo("重新设定起止状态！\r\n")
                if self.start_base_mode == 1:
                    self.set_start_state_to_last_target_cmd()
                else:
                    self.arm_moveit.set_start_state_to_current_state()
                self.target_pose.header.stamp = rospy.Time.now()  # 设定时间为当前时间
                self.arm_moveit.set_joint_value_target(self.target_pose,self.end_effector_link,self.precise_pose)
                self.new_joint_target = self.arm_moveit.get_joint_value_target()  # moveit仅会记录一个最新的target
                # loginfo("开始规划路径……!\r\n")
                plan_success, traj, planning_time, error_code = self.arm_moveit.plan()
                if(plan_success):
                    # loginfo("规划成功，开始执行!\r\n")
                    self.arm_moveit.execute(traj,wait=self.wait_flag)  # 因为上面调用了stop，故理论上无需等待，否则stop将不起作用必然将等待完成；但是不等待机械臂会明显表现出卡顿，因为plan频率过低。
                    loginfo("MoveIt执行完毕。\r\n")
                    # self.pose_info_log(nowtime=True)
                    # 只有执行成功后才更新起点目标为最新目标
                    self.last_joint_target = self.new_joint_target
                else: # 若执行失败，则保留上次的起点目标，同时积分进行回退
                    loginfo("MoveIt规划失败。\r\n")
                    if self.use_integral:
                        # 应将上次的扣除，避免目标偏离当前位置
                        self.integral_sum_rpy = [self.integral_sum_rpy[i]-self.tf_angles[i] for i in range(3)]
                        self.integral_sum_xyz = [self.integral_sum_xyz[i]-self.tf_trans[i]  for i in range(3)]
                self.new_feedback = False  # 执行完再设置为F，可以保证数据的实时性


if __name__ == "__main__":
    ros_node = MoveitTagFollowNode()
    # 启动MoveIt跟踪线程
    spin_thread = threading.Thread(target = ros_node.MoveitFollow)
    spin_thread.setDaemon(True)
    spin_thread.start()
    # 主循环
    rospy.spin()
