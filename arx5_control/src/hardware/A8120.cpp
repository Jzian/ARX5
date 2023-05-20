#include "hardware/A8120.h"
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f



union RV_TypeConvert1
{
	float to_float;
	int to_int;
	unsigned int to_uint;
	uint8_t buf[4];
}rv_type_convert1;

union RV_TypeConvert
{
	float to_float;
	int to_int;
	unsigned int to_uint;
	uint8_t buf[4];
}rv_type_convert;


MotorCommFbd motor_comm_fbd;
OD_Motor_Msg rv_motor_msg[8];

void MotorIDReading(uint8_t* pData, uint32_t* pCanID, uint8_t* pDataBufferLen)
{
	*pCanID = 0x7FF;
	*pDataBufferLen = 4;
    pData[0]=0xFF;
	pData[1]=0xFF;
	pData[2]=0x00;
	pData[3]=0x82;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
param_cmd:1~9
*/
void GetMotorParameter(uint16_t motor_id, uint8_t param_cmd, uint8_t* pData, uint32_t* pCanID, uint8_t* pDataBufferLen)
{
    *pCanID = motor_id;
    *pDataBufferLen = 2;
    pData[0] = 0xE0;
    pData[1] = param_cmd;
}

//MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void MotorSetting(uint16_t motor_id,uint8_t cmd,uint8_t* Data,uint32_t* canID)
{
    *canID = motor_id;
	if(cmd==0) return;
    Data[0] = motor_id>>8;
    Data[1] = motor_id&0xff;
    Data[2] = 0x00;
    Data[3] = cmd;
    
}
// This function use in ask communication mode.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void send_motor_ctrl_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor, uint8_t* Data, uint32_t* canID)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;

    *canID = motor_id;
  
    if (kp > KP_MAX){kp = KP_MAX;}
    else if (kp < KP_MIN){kp = KP_MIN;}
    if (kd > KD_MAX){kd = KD_MAX;}
    else if (kd < KD_MIN){kd = KD_MIN;}
    if (pos > POS_MAX){pos = POS_MAX;}
    else if (pos < POS_MIN) {pos = POS_MIN;}
    if (spd > SPD_MAX){spd = SPD_MAX;}
    else if (spd < SPD_MIN){spd = SPD_MIN;}
    if (tor > T_MAX){tor = T_MAX;}
    else if (tor < T_MIN){tor = T_MIN;}

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);

    Data[0] = 0x00 | (kp_int >> 7);                             // kp5
    Data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    Data[2] = kd_int & 0xFF;
    Data[3] = pos_int >> 8;
    Data[4] = pos_int & 0xFF;
    Data[5] = spd_int >> 4;
    Data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    Data[7] = tor_int & 0xff;
    // ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>send_motor_ctrl_cmd>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

}
void send_motor_position(uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status, uint8_t* Data, uint32_t* canID)
{
    *canID = motor_id;
  	if(spd>18000) spd=18000;
	if(cur>3000) cur=3000;
	if(ack_status>3) 
		return;
    // ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>send_motor_ctrl_cmd>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    rv_type_convert1.to_float=pos;
	Data[0]=0x20|(rv_type_convert1.buf[3]>>3);
	Data[1]=(rv_type_convert1.buf[3]<<5)|(rv_type_convert1.buf[2]>>3);
	Data[2]=(rv_type_convert1.buf[2]<<5)|(rv_type_convert1.buf[1]>>3);
	Data[3]=(rv_type_convert1.buf[1]<<5)|(rv_type_convert1.buf[0]>>3);
	Data[4]=(rv_type_convert1.buf[0]<<5)|(spd>>10);
	Data[5]=(spd&0x3FC)>>2;
	Data[6]=(spd&0x03)<<6|(cur>>6);
	Data[7]=(cur&0x3F)<<2|ack_status;
}


void RV_can_data_repack(uint32_t msgID, uint8_t* Data, int32_t databufferlen, uint8_t comm_mode)
{
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    if (msgID == 0x7FF)
    {
        if (Data[2] != 0x01) // determine whether it is a motor feedback instruction
            return;                     // it is not a motor feedback instruction
        if ((Data[0] == 0xff) && (Data[1] == 0xFF))
        {
            motor_comm_fbd.motor_id = Data[3] << 8 | Data[4];
            motor_comm_fbd.motor_fbd = 0x01;
            rv_motor_msg[motor_comm_fbd.motor_id-1].buildConnect = true;
        }
        else if ((Data[0] == 0x80) && (Data[1] == 0x80)) // inquire failed
        {
            motor_comm_fbd.motor_id = 0;
            motor_comm_fbd.motor_fbd = 0x80;
        }
        else if ((Data[0] == 0x7F) && (Data[1] == 0x7F)) // reset ID succeed
        {
            motor_comm_fbd.motor_id = 1;
            motor_comm_fbd.motor_fbd = 0x05;
        }
        else
        {
            motor_comm_fbd.motor_id = Data[0] << 8 | Data[1];
            motor_comm_fbd.motor_fbd = Data[3];
        }
    }
    else if (comm_mode == 0x00) // Response mode
    {
        ack_status = Data[0] >> 5;
        motor_id_t = msgID - 1;
        rv_motor_msg[motor_id_t].motor_id = motor_id_t;
        rv_motor_msg[motor_id_t].error = Data[0] & 0x1F;
        if (ack_status == 1) // response frame 1
        {
            pos_int = Data[1] << 8 | Data[2];
            spd_int = Data[3] << 4 | (Data[4] & 0xF0) >> 4;
            cur_int = (Data[4] & 0x0F) << 8 | Data[5];

            rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
            // ROS_INFO("\033[36mresponse frame 1 angle_actual_rad = %f  \033[0m", rv_motor_msg[motor_id_t].angle_actual_rad);
            rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
            rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12);
            rv_motor_msg[motor_id_t].temperature = (Data[6] - 50) / 2;
        }
        else if (ack_status == 2) // response frame 2
        {
            rv_type_convert.buf[0] = Data[4];
            rv_type_convert.buf[1] = Data[3];
            rv_type_convert.buf[2] = Data[2];
            rv_type_convert.buf[3] = Data[1];
            rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
            
            rv_motor_msg[motor_id_t].current_actual_int = Data[5] << 8 | Data[6];
            rv_motor_msg[motor_id_t].temperature = (Data[7] - 50) / 2;
            rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
        }
        else if (ack_status == 3) // response frame 3
        {
            rv_type_convert.buf[0] = Data[4];
            rv_type_convert.buf[1] = Data[3];
            rv_type_convert.buf[2] = Data[2];
            rv_type_convert.buf[3] = Data[1];
            rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
            rv_motor_msg[motor_id_t].current_actual_int = Data[5] << 8 | Data[6];
            rv_motor_msg[motor_id_t].temperature = (Data[7] - 50) / 2;
            rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
        }
        else if (ack_status == 4) // response frame 4
        {
            if (databufferlen != 3)
                return;
            motor_comm_fbd.INS_code = Data[1];
            motor_comm_fbd.motor_fbd = Data[2];
        }
        else if (ack_status == 5) // response frame 5
        {
            motor_comm_fbd.INS_code = Data[1];
            if (motor_comm_fbd.INS_code == 1 & databufferlen == 6) // get position
            {
                rv_type_convert.buf[0] = Data[5];
                rv_type_convert.buf[1] = Data[4];
                rv_type_convert.buf[2] = Data[3];
                rv_type_convert.buf[3] = Data[2];
                rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].getInitAngle = true;
            }
            else if (motor_comm_fbd.INS_code == 2 & databufferlen == 6) // get speed
            {
                rv_type_convert.buf[0] = Data[5];
                rv_type_convert.buf[1] = Data[4];
                rv_type_convert.buf[2] = Data[3];
                rv_type_convert.buf[3] = Data[2];
                rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
            }
            else if (motor_comm_fbd.INS_code == 3 & databufferlen == 6) // get current
            {
                rv_type_convert.buf[0] = Data[5];
                rv_type_convert.buf[1] = Data[4];
                rv_type_convert.buf[2] = Data[3];
                rv_type_convert.buf[3] = Data[2];
                rv_motor_msg[motor_id_t].current_actual_float = rv_type_convert.to_float;
            }
            else if (motor_comm_fbd.INS_code == 4 & databufferlen == 6) // get power
            {
                rv_type_convert.buf[0] = Data[5];
                rv_type_convert.buf[1] = Data[4];
                rv_type_convert.buf[2] = Data[3];
                rv_type_convert.buf[3] = Data[2];
                rv_motor_msg[motor_id_t].power = rv_type_convert.to_float;
            }
            else if (motor_comm_fbd.INS_code == 5 & databufferlen == 4) // get acceleration
            {
                rv_motor_msg[motor_id_t].acceleration = Data[2] << 8 | Data[3];
            }
            else if (motor_comm_fbd.INS_code == 6 & databufferlen == 4) // get linkage_KP
            {
                rv_motor_msg[motor_id_t].linkage_KP = Data[2] << 8 | Data[3];
            }
            else if (motor_comm_fbd.INS_code == 7 & databufferlen == 4) // get speed_KI
            {
                rv_motor_msg[motor_id_t].speed_KI = Data[2] << 8 | Data[3];
            }
            else if (motor_comm_fbd.INS_code == 8 & databufferlen == 4) // get feedback_KP
            {
                rv_motor_msg[motor_id_t].feedback_KP = Data[2] << 8 | Data[3];
            }
            else if (motor_comm_fbd.INS_code == 9 & databufferlen == 4) // get feedback_KD
            {
                rv_motor_msg[motor_id_t].feedback_KD = Data[2] << 8 | Data[3];
            }
        }
    }
    else if (comm_mode == 0x01) // automatic feedback mode
    {

        motor_id_t = msgID - 0x205;
        rv_motor_msg[motor_id_t].angle_actual_int = (uint16_t)(Data[0] << 8 | Data[1]);
        // ROS_INFO("\033[36mautomatic feedback mode angle_actual_rad = %d  \033[0m", rv_motor_msg[motor_id_t].angle_actual_int);
        rv_motor_msg[motor_id_t].speed_actual_int = (int16_t)(Data[2] << 8 | Data[3]);
        rv_motor_msg[motor_id_t].current_actual_int = (Data[4] << 8 | Data[5]);
        rv_motor_msg[motor_id_t].temperature = Data[6];
        rv_motor_msg[motor_id_t].error = Data[7];
    }
}
