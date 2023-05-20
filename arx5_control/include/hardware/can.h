/*
 * @Author: jia-yf19
 * @Date: 2022-03-27 19:50:45
 * @LastEditTime: 2022-03-31 18:42:54
 * @Description: 一切can收发
 * @FilePath: /src/rcbigcar/src/mpc_ctr/hardware/can.h
 */

#ifndef _CAN_H_
#define _CAN_H_

#include <stdint.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "../libcan/SocketCAN.h"
#include "A8120.h"

// //所有角度信息 角度和角加速度
typedef struct
{
    float imu[3];  //单位 rad    顺序yaw pitch roll
    float last_imu[3]; //上一时刻的imu角度数据
    float gyro[3]; //单位 rad/s  顺序pitch  roll  yaw
} IMU_Float_t;

typedef struct{
    int8_t   temperature;
    int16_t	 speed_rpm;
    int16_t  real_current;
    uint16_t position;
    int8_t   round_cnt;
    float    total_angle;
    float    total_angle_last;
	
}m_rmd_t;

extern m_rmd_t rmd_9015_01;
extern m_rmd_t rmd_9015_02;

// 负责can收发
class can
{
public:
    can();
    ~can();

    // sensor_msgs::Imu ImuMsg;
    // Motor_measure_t Motor[MOTOR_NUM] = {0};

    void CAN0_ReceiveFrame(can_frame_t *frame);
    void CAN1_ReceiveFrame(can_frame_t *frame);

    void CAN_cmd_readMotorID(void);
    void CAN_cmd_getMotorParam(uint16_t motor_id, uint8_t param_cmd);
    void CAN_cmd_init(uint16_t motor_id,uint8_t cmd);
    void CAN_cmd_position(uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status);

    void CAN_cmd_all(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    // motor id 0x01 ~ 0x04
    void CAN_cmd_fpc1(float kp[4], float kd[4], float pos[4], float spd[4], float tor[4]);
    // motor id 0x05 ~ 0x08
    void CAN_cmd_fpc2(float kp[4], float kd[4], float pos[4], float spd[4], float tor[4]);
    void MotorSetting(uint16_t motor_id,uint8_t cmd);
    // // motor id 0x01 ~ 0x04
    // void CAN_cmd_chassis1(int16_t motor[4]);
    // // motor id 0x05 ~ 0x08
    // void CAN_cmd_chassis2(int16_t motor[4]);
    
private:
    SocketCAN can0_adapter;
    SocketCAN can1_adapter;
};

#endif