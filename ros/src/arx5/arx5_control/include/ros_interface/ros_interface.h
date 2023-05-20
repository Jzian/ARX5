#ifndef _ROS_INTERFACE_H_
#define _ROS_INTERFACE_H_

#include "../hardware/can.h"
#include "../hardware/dbus.h"
#include "../hardware/A8120.h"

#include "ros/ros.h"
#include "arx5_msgs/JointFeedback.h"
#include "arx5_msgs/JointCommand.h"
#include "sensor_msgs/JointState.h"

#include <stdlib.h>

// 电机数量
#define NUM_JOINTS 6

class ros_interface
{
public:
    ros_interface();
    ~ros_interface()=default;

    // CAN总线通信
    can CAN_handle;

    // 力矩增益系数（力矩=力矩*增益系数）
    float torque_gain = 1.1;
    // 力矩最大值
    float torque_max = 15.0;

    // 力矩平滑滤波参数（上一时刻的力矩数据所占比例，取值范围0-1）
    float torque_filter = 0.3;
    // 上一时刻的力矩数据缓存，用于平滑滤波
    float torque_cache[NUM_JOINTS] = {0.0};

    // 位置反馈平滑滤波参数（上一时刻的位置数据所占比例，取值范围0-1）
    float position_feedback_filter = 0.0;
    // 上一时刻的位置反馈数据缓存，用于平滑滤波
    float position_feedback_cache[NUM_JOINTS] = {0.0};

    // 速度反馈平滑滤波参数（上一时刻的速度数据所占比例，取值范围0-1）
    float velocity_feedback_filter = 0.0;
    // 上一时刻的速度反馈数据缓存，用于平滑滤波
    float velocity_feedback_cache[NUM_JOINTS] = {0.0};

    // 关节反馈数据
    arx5_msgs::JointFeedback feedback_msg;
    sensor_msgs::JointState joint_states_msg;
    // 发布关节反馈数据
    void update_joint_feedback();
    // 发布关节控制命令
    void send_joint_command();
    int send_cmd_state = 0;
    bool is_sending = false;
    // 判断是否收到消息
    bool avtivated = false;

    // 关节角度限位
    float angle_lower_bound[7] = {-2.3, -2.9, 0.0, -1.72, -1.35, -1.4, -0.98};
    float angle_upper_bound[7] = {3, -0.02, 2.7, 1.5, 1.35, 1.4, 0.0};

    int motor_ids[6] = {1,2,4,5,6,7};

private:
    // 订阅关节控制消息
    ros::Subscriber _joint_commands_subscriber;
    // 发布关节反馈数据
    ros::Publisher _joint_feedback_publisher;
    ros::Publisher _joint_states_publisher;
    arx5_msgs::JointCommand _last_joint_cmd;
    // 订阅关节控制消息的回调函数
    void _joint_commands_callback(const arx5_msgs::JointCommand::ConstPtr& data);
};

#endif
