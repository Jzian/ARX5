#include "hardware/can.h"

m_rmd_t rmd_9015_01={0};
m_rmd_t rmd_9015_02={0};
m_rmd_t rmd_01={0};

void CAN0_ReceiveHandlerProxy(can_frame_t *frame, void *ptr)
{
    ((can *)ptr)->CAN0_ReceiveFrame(frame);
}

void CAN1_ReceiveHandlerProxy(can_frame_t *frame2, void *ptr)
{
    ((can *)ptr)->CAN1_ReceiveFrame(frame2);
}

can::can()
{
    can0_adapter.reception_handler_data = (void *)this;
    can0_adapter.reception_handler = &CAN0_ReceiveHandlerProxy;
    can0_adapter.open("can0");

    can1_adapter.reception_handler_data = (void *)this;
    can1_adapter.reception_handler = &CAN1_ReceiveHandlerProxy;
    can1_adapter.open("can1");

    ros::Time::init();
}

can::~can()
{
    can0_adapter.close();
    can1_adapter.close();
}

void can::CAN_cmd_readMotorID(void)
{
    can_frame_t frame;
    MotorIDReading(frame.data, &frame.can_id, &frame.can_dlc);
    if (can0_adapter.is_open())
    {
        can0_adapter.transmit(&frame);
    }
    else
    {
        std::cout << "Fail to open can0" << std::endl;
    }
}


void can::CAN_cmd_getMotorParam(uint16_t motor_id, uint8_t param_cmd)
{
    can_frame_t frame;
    GetMotorParameter(motor_id, param_cmd, frame.data, &frame.can_id, &frame.can_dlc);
    if (can0_adapter.is_open())
    {
        can0_adapter.transmit(&frame);
    }
    else
    {
        std::cout << "Fail to open can0" << std::endl;
    }    
}

void can::CAN_cmd_init(uint16_t motor_id,uint8_t cmd)
{   
    
    can_frame_t frame;
    frame.can_dlc = 8;
    MotorSetting(motor_id,cmd);
    if (can0_adapter.is_open())
    {
        can0_adapter.transmit(&frame);
    }
    else
    {
        std::cout << "Fail to open can0" << std::endl;
    }
    
}
void can::CAN_cmd_position(uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status)
{
    can_frame_t frame;
    frame.can_dlc = 8;
    send_motor_position(motor_id,pos,spd,cur,ack_status,frame.data, &frame.can_id);
    if (can0_adapter.is_open())
    {
        can0_adapter.transmit(&frame);
    }
    else
    {
        std::cout << "Fail to open can0" << std::endl;
    }
}

void can::CAN_cmd_all(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    can_frame_t frame;
    frame.can_dlc = 8;
    send_motor_ctrl_cmd(motor_id, kp, kd, pos, spd, tor, frame.data, &frame.can_id);
    if (can0_adapter.is_open())
    {
        can0_adapter.transmit(&frame);
    }
    else
    {
        std::cout << "Fail to open can0" << std::endl;
    }
}

// Motor A8120
// motor id 0x01 ~ 0x04
void can::CAN_cmd_fpc1(float kp[4], float kd[4], float pos[4], float spd[4], float tor[4])
{
    for (int i = 0; i < 4; i++)
    {
        can_frame_t frame;
        frame.can_dlc = 8;
        send_motor_ctrl_cmd(i+1, kp[i], kd[i], pos[i], spd[i], tor[i], frame.data, &frame.can_id);
        if (can0_adapter.is_open())
        {
            can0_adapter.transmit(&frame);
        }
        else
        {
            std::cout << "Fail to open can0" << std::endl;
        }
    }
}

// Motor A8120 
// motor id 0x05 ~ 0x08
void can::CAN_cmd_fpc2(float kp[4], float kd[4], float pos[4], float spd[4], float tor[4])
{
    for (int i = 0; i < 4; i++)
    {
        can_frame_t frame;
        frame.can_dlc = 8;
        send_motor_ctrl_cmd(i+5, kp[i], kd[i], pos[i], spd[i], tor[i], frame.data, &frame.can_id);
        if (can0_adapter.is_open())
        {
            can0_adapter.transmit(&frame);
        }
        else
        {
            std::cout << "Fail to open can0" << std::endl;
        }
    }
}


// // Motor 8120
// // motor id 0x01 ~ 0x04
// void can::CAN_cmd_chassis1(int16_t motor[4])
// {
//     can_frame_t frame;
//     frame.can_id = 0x1FF;
//     frame.can_dlc = 8;
//     for (int i = 0; i < 4; i++)
//     {
//         frame.data[2 * i] = motor[i] >> 8;
//         frame.data[2 * i + 1] = motor[i] & 0xFF;
//     }
//     if (can0_adapter.is_open())
//     {
//         can0_adapter.transmit(&frame);
//     }
//     else
//     {
//         std::cout << "Fail to open can0" << std::endl;
//     }
// }

// // Motor 8120
// // motor id 0x05 ~ 0x08
// void can::CAN_cmd_chassis2(int16_t motor[4])
// {
//     can_frame_t frame;
//     frame.can_id = 0x2FF;
//     frame.can_dlc = 8;
//     for (int i = 0; i < 4; i++)
//     {
//         frame.data[2 * i] = motor[i] >> 8;
//         frame.data[2 * i + 1] = motor[i] & 0xFF;
//     }
//     if (can0_adapter.is_open())
//     {
//         can0_adapter.transmit(&frame);
//     }
//     else
//     {
//         std::cout << "Fail to open can0" << std::endl;
//     }
// }

/**
 * @description: 收到can0的消息
 */
void can::CAN0_ReceiveFrame(can_frame_t *frame)
{

    switch (frame->can_id)
    {
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
    case 0x08:
    case 0x7FF:
    {
        RV_can_data_repack(frame->can_id, frame->data, frame->can_dlc, 0);
        break;
    }
    case 0x205:
    case 0x206:
    case 0x207:
    case 0x208:
    case 0x209:
    case 0x20A:
    case 0x20B:
    case 0x20C:
    {
        // // 电机
        // uint8_t i = frame->can_id - 0x205;
        // if (Motor[i].msg_cnt < 10)
        // {
        //     get_motor_offset(&Motor[i], frame->data);
        //     Motor[i].msg_cnt++;
        // }

        // get_motor_measure(&Motor[i], frame->data);

        // // Motor[i].msg_cnt++ <= 10 ? get_motor_offset(&Motor[i], frame->data) : get_motor_measure(&Motor[i], frame->data);
        break;
    }
    
   
    // case 0x141:
    // {
    //     rmd_9015_01.speed_rpm=((int16_t)(frame->data[5]<<8) |(frame->data[4]))/57.3f;
    //     rmd_9015_01.position =((int16_t)(frame->data[7]<<8) |(frame->data[6]));	
    //     break;
    // }
    case 0x141:
    {
        rmd_01.total_angle_last=rmd_01.position;
        rmd_01.speed_rpm=((int16_t)(frame->data[5]<<8) |(frame->data[4]))/57.3f;
        rmd_01.position=((int16_t)(frame->data[7]<<8) |(frame->data[6]));	
        rmd_01.real_current=((int16_t)(frame->data[2]<<8) |(frame->data[3]));
        if (rmd_01.position - rmd_01.total_angle_last > 8192)
            rmd_01.round_cnt--;
        else if (rmd_01.position - rmd_01.total_angle_last < -8192)
            rmd_01.round_cnt++;
        rmd_01.total_angle = rmd_01.round_cnt * 16384 + rmd_01.position;					
        break;
    }
    case 0x142:
    {
        rmd_9015_02.speed_rpm=((int16_t)(frame->data[5]<<8) |(frame->data[4]))/57.3f;
        rmd_9015_02.position =((int16_t)(frame->data[7]<<8) |(frame->data[6]));	
        break;
    }   
   
    case 0x501:
    {
        // std::cout << "# frame can_id= " << frame->can_id << std::endl;
        // std::cout << "# frame data0-3  = " << frame->data[0] << " " << frame->data[1] << " " << frame->data[2] << " " << frame->data[3] << " " << std::endl;
        // std::cout << "# frame data4-7  = " << frame->data[4] << " " << frame->data[5] << " " << frame->data[6] << " " << frame->data[7] << " " << std::endl;
        // IMU
        // float tmp[2];
        // memcpy((uint8_t *)(&tmp[0]), (uint8_t *)(&frame->data[0]), 4);
        // memcpy((uint8_t *)(&tmp[1]), (uint8_t *)(&frame->data[4]), 4);
        // IMU.imu[0] = tmp[0] * 0.0174533;
        // IMU.imu[1] = tmp[1] * 0.0174533;
        break;
    }
    case 0x502:
    {
        // std::cout << "# frame can_id= " << frame->can_id << std::endl;
        // std::cout << "# frame data0-3  = " << frame->data[0] << " " << frame->data[1] << " " << frame->data[2] << " " << frame->data[3] << " " << std::endl;
        // std::cout << "# frame data4-7  = " << frame->data[4] << " " << frame->data[5] << " " << frame->data[6] << " " << frame->data[7] << " " << std::endl;
        // // Gyro
        // memcpy((uint8_t *)(&IMU.gyro[0]), (uint8_t *)(&frame->data[0]), 4);
        // memcpy((uint8_t *)(&IMU.gyro[1]), (uint8_t *)(&frame->data[4]), 4);
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @description: 收到can0的消息
 */
void can::CAN1_ReceiveFrame(can_frame_t *frame)
{
    // switch (frame->can_id)
    // {
    //     break;
    // }
}


//  01  02  03零点设置
void can::MotorSetting(uint16_t motor_id,uint8_t cmd)
{
    can_frame_t frame;
    frame.can_id = 0x7FF;
    frame.can_dlc = 4;
	if(cmd==0) return;

	frame.data[0]=motor_id>>8;
	frame.data[1]=motor_id&0xff;
	frame.data[2]=0x00;
	frame.data[3]=cmd;

    if (can0_adapter.is_open())
    {
        can0_adapter.transmit(&frame);
    }
    else
    {
        std::cout << "Fail to open can0" << std::endl;
    }

}