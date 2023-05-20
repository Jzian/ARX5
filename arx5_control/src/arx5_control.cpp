#include "ros_interface/ros_interface.h"

int main(int argc, char **argv)
{
    // 开启ROS接口
    ros::init(argc, argv, "arx5_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(200);
    ros_interface ri;
    while(ros::ok())
    {
        ri.send_cmd_state = 0;
        // 时序频率控制
        ros::spinOnce();
        ros::Duration(0.001).sleep();
        // 发布控制命令(200Hz)
        if (ri.send_cmd_state == 0) ri.send_joint_command();
        //发布关节反馈数据
        ri.update_joint_feedback();
        loop_rate.sleep();
    }

    return 0;
}
