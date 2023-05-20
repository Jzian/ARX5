#ifndef _A8120_H_
#define _A8120_H_

#define param_get_pos 0x01
#define param_get_spd 0x02
#define param_get_cur 0x03
#define param_get_pwr 0x04
#define param_get_acc 0x05
#define param_get_lkgKP 0x06
#define param_get_spdKI 0x07
#define param_get_fdbKP 0x08
#define param_get_fdbKD 0x09

#define comm_ack 0x00
#define comm_auto 0x01

#include <stdint.h>
#include "math_ops.h"
#include <ros/ros.h>
typedef struct
{
    uint16_t motor_id;
    uint8_t INS_code;  // instruction code.
    uint8_t motor_fbd; // motor CAN communication feedback.
} MotorCommFbd;

typedef struct
{
    int16_t angle_actual_int = 0;
    int16_t angle_desired_int = 0;
    int16_t speed_actual_int = 0;
    int16_t speed_desired_int = 0;
    int16_t current_actual_int = 0;
    int16_t current_desired_int = 0;
    float speed_actual_rad = 0;
    float speed_desired_rad = 0.0;
    float angle_actual_rad = 0.0;
    float angle_desired_rad = 0.0;
    uint16_t motor_id = 0;
    uint8_t temperature = 0;
    uint8_t error = 0;
    float angle_actual_float = 0.0;
    float speed_actual_float = 0.0;
    float current_actual_float = 0.0;
    float angle_desired_float = 0.0;
    float speed_desired_float = 0.0;
    float current_desired_float = 0.0;
    float power = 0.0;
    uint16_t acceleration = 0;
    uint16_t linkage_KP = 0;
    uint16_t speed_KI = 0;
    uint16_t feedback_KP = 0;
    uint16_t feedback_KD = 0;
    bool buildConnect = 0;
    bool getInitAngle = 0;
} OD_Motor_Msg;

extern OD_Motor_Msg rv_motor_msg[8];
extern uint16_t motor_id_check;

void MotorSetting(uint16_t motor_id,uint8_t cmd);
void MotorIDReading(uint8_t* pData, uint32_t* pCanID, uint8_t* pDataBufferLen);
void GetMotorParameter(uint16_t motor_id, uint8_t param_cmd, uint8_t* pData, uint32_t* pCanID, uint8_t* pDataBufferLen);
void send_motor_ctrl_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor, uint8_t* Data, uint32_t* canID);
void RV_can_data_repack(uint32_t msgID, uint8_t* Data, int32_t databufferlen, uint8_t comm_mode);
void send_motor_position(uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status, uint8_t* Data, uint32_t* canID);
#endif
