#include "ros_interface/ros_interface.h"
extern OD_Motor_Msg rv_motor_msg[8];

ros_interface::ros_interface()
{
    // 标定
    // for(int i=0; i<NUM_JOINTS; i++)
    // {
    //     CAN_handle.CAN_cmd_init(i+1,0x03);
    // }

    // 开启ROS节点
    ros::NodeHandle nh_private;
    _joint_commands_subscriber = nh_private.subscribe<arx5_msgs::JointCommand>("arx5/joint_command", 10, &ros_interface::_joint_commands_callback, this);
    _joint_feedback_publisher  = nh_private.advertise<arx5_msgs::JointFeedback>("arx5/joint_feedback", 10);
    _joint_states_publisher  = nh_private.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // 从ROS中获取参数
    nh_private.getParam("torque_gain",torque_gain);
    nh_private.getParam("torque_max",torque_max);
    nh_private.getParam("torque_filter",torque_filter);

    //初始化控制命令
    _last_joint_cmd.mode = "None";
}

void ros_interface::send_joint_command()
{
    if (is_sending) return;
    else is_sending = true;

    if (_last_joint_cmd.mode == "None")  // 初始电机无力模式
    {
        for(int i=0; i<NUM_JOINTS; i++)
        {
            CAN_handle.CAN_cmd_position(motor_ids[i], 0, 0, 0, 2);
        }
    }
    else if (_last_joint_cmd.mode == "torque")
    {
        // 通过CAN总线向电机发送控制指令
        float kp_012 = 210;
        float kd_012 = 15;
        float kp_34 = 40;
        float kd_34 = 3;
        float kp_end = 20;
        float kd_end = 1;

        float eff = 0;
        float spd = 0;

        CAN_handle.CAN_cmd_all(1, kp_012, kd_012, _last_joint_cmd.position[0], spd, eff);
        CAN_handle.CAN_cmd_all(2, kp_012, kd_012, _last_joint_cmd.position[1], spd, eff);
        CAN_handle.CAN_cmd_all(4, kp_012, kd_012, _last_joint_cmd.position[2], spd, eff);
        CAN_handle.CAN_cmd_all(5, kp_34,  kd_34,  _last_joint_cmd.position[3], spd, eff);
        CAN_handle.CAN_cmd_all(6, kp_34,  kd_34,  _last_joint_cmd.position[4], spd, eff);
        CAN_handle.CAN_cmd_all(7, kp_end, kd_end, _last_joint_cmd.position[5], spd, eff);
    }
    else if(_last_joint_cmd.mode == "position")
    {
        int current = 0;
        for(int i=0; i<NUM_JOINTS; i++)
        {
            // 速度限幅
            if (_last_joint_cmd.velocity[i] > 2*M_PI) _last_joint_cmd.velocity[i] = 2*M_PI;
            else if (_last_joint_cmd.velocity[i] < 0) _last_joint_cmd.velocity[i] = 0;
            // 命令发布
            if (i==2)
            {
                if (_last_joint_cmd.effort[i] == 1) current = 0;
                else current = 800;
                CAN_handle.CAN_cmd_position(motor_ids[i], _last_joint_cmd.position[i]*180/M_PI,_last_joint_cmd.velocity[i]*180/M_PI, current, 2);
            }
            else if (i>2)
            {
                if (_last_joint_cmd.effort[i] == 1) current = 0;
                else current = 200;
                CAN_handle.CAN_cmd_position(motor_ids[i], _last_joint_cmd.position[i]*180/M_PI,_last_joint_cmd.velocity[i]*180/M_PI, current, 2);
            }
            else
            {
                if (_last_joint_cmd.effort[i] == 1) current = 0;
                else current = 1000;
                CAN_handle.CAN_cmd_position(motor_ids[i], _last_joint_cmd.position[i]*180/M_PI,_last_joint_cmd.velocity[i]*180/M_PI, current, 2);
            }
        }
    }
    else
    {
        ROS_ERROR("ERROR in arx5_msg.msg.JointCommand.mode. The mode you choose is not supported. Please try torque/PD/position/test.");
    }
    is_sending = false;
}

//根据反馈的控制命令进行电机控制（角度单位统一为rad）
void ros_interface::_joint_commands_callback(const arx5_msgs::JointCommand::ConstPtr& data)
{
    send_cmd_state = 1;
    _last_joint_cmd = *data;
    _last_joint_cmd.position[1] *= -1;
    if(data->mode == "torque")
    {
        // 力矩控制模式
        float effort[NUM_JOINTS] = {0.0};
        // 遍历每一个电机
        for(int i=0; i<NUM_JOINTS; i++)
        {
            // 对力矩进行增益处理
            effort[i] = data->effort[i]*torque_gain;
            // 对力矩进行限幅处理
            if(effort[i] > torque_max)
            {
                effort[i] = torque_max;
            }
            else if(effort[i] < -torque_max)
            {
                effort[i] = -torque_max;
            }
            // 结合上一时刻的力矩数据，对力矩进行滤波处理
            effort[i] = effort[i] * (1 - torque_filter) + torque_cache[i] * torque_filter;
            // 储存上一时刻的理据数据
            torque_cache[i] = effort[i];
            _last_joint_cmd.effort[i] = effort[i];
        }
    }
    send_joint_command();
}

void ros_interface::update_joint_feedback()
{
    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.header.frame_id = "arx5";
    for(int i=0; i<NUM_JOINTS+1; i++)
    {
        feedback_msg.id.push_back(i+1);
        if(_last_joint_cmd.mode == "position" || _last_joint_cmd.mode == "None")
        {
            feedback_msg.position.push_back(rv_motor_msg[i].angle_actual_float/180*M_PI);
            feedback_msg.current.push_back(rv_motor_msg[i].current_actual_float);
            feedback_msg.temperature.push_back(rv_motor_msg[i].temperature);
        }
        else if(_last_joint_cmd.mode == "torque" || _last_joint_cmd.mode == "PD" || _last_joint_cmd.mode == "test")
        {
            float current_position = rv_motor_msg[i].angle_actual_rad * (1 - position_feedback_filter) + position_feedback_cache[i] * position_feedback_filter;
            position_feedback_cache[i] = current_position;
            feedback_msg.position.push_back(current_position);

            float current_velocity = rv_motor_msg[i].speed_actual_rad * (1 - velocity_feedback_filter) + velocity_feedback_cache[i] * velocity_feedback_filter;
            velocity_feedback_cache[i] = current_velocity;
            feedback_msg.velocity.push_back(current_velocity);

            feedback_msg.current.push_back(rv_motor_msg[i].current_actual_float);
            feedback_msg.temperature.push_back(rv_motor_msg[i].temperature);
        }
    }

    _joint_feedback_publisher.publish(feedback_msg);

    // 给joint_state_publisher用的消息
    joint_states_msg.header.stamp = ros::Time::now();
    joint_states_msg.header.frame_id = "arx5";

    joint_states_msg.name.push_back("base_link_to_link1");
    joint_states_msg.position.push_back(feedback_msg.position[0]);

    joint_states_msg.name.push_back("link1_to_link2");
    joint_states_msg.position.push_back(-feedback_msg.position[1]);

    joint_states_msg.name.push_back("link2_to_link3");
    joint_states_msg.position.push_back(feedback_msg.position[3]);

    joint_states_msg.name.push_back("link3_to_link4");
    joint_states_msg.position.push_back(feedback_msg.position[4]);

    joint_states_msg.name.push_back("link4_to_link5");
    joint_states_msg.position.push_back(feedback_msg.position[5]);

    joint_states_msg.name.push_back("link5_to_gripper_link1");
    joint_states_msg.position.push_back(feedback_msg.position[6]);

    _joint_states_publisher.publish(joint_states_msg);

    // 清空数据
    feedback_msg.id.clear();
    feedback_msg.position.clear();
    feedback_msg.velocity.clear();
    feedback_msg.current.clear();
    feedback_msg.temperature.clear();

    // 清空数据
    joint_states_msg.name.clear();
    joint_states_msg.position.clear();
}
