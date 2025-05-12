
#include "can_protocol.h"
#include <iostream>
//#include "v_ros_msg_interface.h"
using namespace std;

/********************************************控制报文封装函数 begin**********************************************/

//档位信号请求函数
bool frame_encapsulation_IDA1(const struct MSG_IDA1& msg, struct can_frame& frame) {
    memset(&frame, 0x00, sizeof(frame));                                     //清空消息缓存区
    frame.can_id = IDA1;                                                  //设置消息ID
    frame.can_dlc = 8;                                                      //设置数据字节数
    frame.data[0] = msg.Gear_Enb;
    frame.data[1] = msg.Gear_Shift_Req;
    frame.data[2] = msg.IPC_ModeCtrl;
    frame.data[3] = msg.IPC_Stop_Eme;

    return true;
}

//转向信号请求函数
bool frame_encapsulation_IDA2(const struct MSG_IDA2& msg, struct can_frame& frame) {
    memset(&frame, 0x00, sizeof(frame));                                     //清空消息缓存区
    frame.can_id = 0xA2;                                                  //设置消息ID
    frame.can_dlc = 8;                                                      //设置数据字节数
    frame.data[0] = msg.Steering_Enb;
    int16_t temp_Steering_Pos_Req = static_cast<uint16_t>(msg.Steering_Pos_Req);
    temp_Steering_Pos_Req = temp_Steering_Pos_Req > 28 ? 28 : (temp_Steering_Pos_Req < -28 ? -28 : temp_Steering_Pos_Req); //限定在 -28~28
    frame.data[1] = static_cast<uint8_t>(temp_Steering_Pos_Req & 0x00ff);
    frame.data[2] = static_cast<uint8_t>((temp_Steering_Pos_Req & 0xff00) >> 8);
    frame.data[3] = msg.Steeringmode;

    return true;
}
//驱动扭矩请求函数
bool frame_encapsulation_IDA3(const struct MSG_IDA3& msg, struct can_frame& frame) {
    memset(&frame, 0x00, sizeof(frame));                                     //清空消息缓存区
    frame.can_id = 0xA3;                                                  //设置消息ID
    frame.can_dlc = 8;                                                      //设置数据字节数
    frame.data[0] = msg.Drive_Enb;
    int16_t temp_Drive_Tq_Req = static_cast<uint16_t>(msg.Drive_Tq_Req);
    temp_Drive_Tq_Req = temp_Drive_Tq_Req > 2200 ? 2200 : temp_Drive_Tq_Req; //限定在 0~2200
    frame.data[1] = static_cast<uint8_t>(temp_Drive_Tq_Req & 0x00ff);
    frame.data[2] = static_cast<uint8_t>((temp_Drive_Tq_Req & 0xff00) >> 8);

    return true;
}

//刹车信号请求函数
bool frame_encapsulation_IDA4(const struct MSG_IDA4& msg, struct can_frame& frame) {
    memset(&frame, 0x00, sizeof(frame));                                     //清空消息缓存区
    frame.can_id = 0xA4;                                                  //设置消息ID
    frame.can_dlc = 8;                                                      //设置数据字节数
    frame.data[0] = msg.Brake_Enb;
    int16_t temp_Brake_Tq_Req = static_cast<uint16_t>(msg.Brake_Tq_Req);
    temp_Brake_Tq_Req = temp_Brake_Tq_Req > 300 ? 300 : temp_Brake_Tq_Req; //限定在 0~300
    frame.data[1] = static_cast<uint8_t>(temp_Brake_Tq_Req & 0x00ff);
    frame.data[2] = static_cast<uint8_t>((temp_Brake_Tq_Req & 0xff00) >> 8);
    return true;
}

/********************************************状态报文解析函数 begin**********************************************/

//车辆驱动状态反馈解析//车辆常规状态反馈函数，包括速度、档位、当前力矩
void frame_parsing_IDC1(const unsigned char* frame_data, struct MSG_IDC1& msg){
    msg.Vehicle_Spd = (static_cast<uint16_t>(frame_data[1] << 8) + static_cast<uint16_t>(frame_data[0])) * 0.1;
    msg.Gear_Pos = frame_data[4];
    msg.Torque_fbk = frame_data[5];
    // static int lastIDC1CheckSum;
    msg.CheckSum_Byte = frame_data[7];
    //检验是否是有效帧
    // if(msg.CheckSum_Byte - lastIDC1CheckSum)
    return ;
}

//电池状态反馈函数，包括电量、电压、电流
void frame_parsing_IDC2(const unsigned char* frame_data, struct MSG_IDC2& msg){
    msg.SOC = frame_data[0];
    msg.Bat_Vol = (static_cast<uint16_t>(frame_data[2] << 8) + static_cast<uint16_t>(frame_data[2])) * 0.1;
    msg.Bat_Discharge_Cur = frame_data[3];
    msg.CheckSum_Byte = frame_data[7];
    return ;
}

//车辆控制状态反馈函数，包括档位使能、转向使能、驱动使能、制动使能、驻车使能、人工接管标志
void frame_parsing_IDC3(const unsigned char* frame_data, struct MSG_IDC3& msg){
    msg.Gear_Enb_fbk = frame_data[0];
    msg.Steering_Enb_fbk = frame_data[1];
    msg.Drive_Enb_fbk = frame_data[2];
    msg.Brake_Enb_fbk = frame_data[3];
    msg.Prk_Enb_fbk = frame_data[4];
    msg.RC_Takeover_flg = frame_data[5];
    msg.CheckSum_Byte = frame_data[7];

    return ;
}

//车辆常规状态反馈函数，包括左右轮有效位、左右轮转速（r/min）
void frame_parsing_IDC4(const unsigned char* frame_data, struct MSG_IDC4& msg){
    msg.Fr_Spd_Valid = frame_data[4];
    msg.Fr_Wheel_Spd = frame_data[5] * 2;
    msg.Rr_Spd_Valid = frame_data[6];
    msg.Rr_Wheel_spd = frame_data[7] * 2;
    return ;

}
//车辆常规状态反馈函数，包括左右电机转速（r/min）、遥控器紧急停车标志位
void frame_parsing_IDC5(const unsigned char* frame_data, struct MSG_IDC5& msg){
    msg.Left_Motor_spd = static_cast<uint16_t>(frame_data[1] << 8) + static_cast<uint16_t>(frame_data[0]);
    msg.Rcontrol_Stop_Emcy = frame_data[2];
    msg.Right_Motor_spd = static_cast<uint16_t>(frame_data[4] << 8) + static_cast<uint16_t>(frame_data[3]);
    return ;
}

//车辆故障状态反馈函数，包括故障码及对应故障状态
void frame_parsing_IDE1(const unsigned char* frame_data, struct MSG_IDE1& msg){
    msg.Error_code = frame_data[0];
    msg.Code_Num = frame_data[1];
    msg.CheckSum_Byte = frame_data[7];
    return ;
}

