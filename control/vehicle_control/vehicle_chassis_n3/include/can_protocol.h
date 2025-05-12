//********************************************************************************************
//新小底盘车（pix普朗克新版）
//**********************************************************************************************

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstdio>
#include <cstring>
#include <cstdint>

//-------------------------------------------------------------控制报文-------------------------------------------------------------------------------

struct MSG_IDA1{
    uint8_t Gear_Enb;   //IPC发出挡位使能信号后，给出的挡位信息才有效 "00：未使能；01：使能；"
    uint8_t Gear_Shift_Req; //挡位切换请求 "01：驻车；02：空挡；03：前进；04：后退；05：无效；"
    uint8_t IPC_ModeCtrl;   //IPC下发模式切换指令 "00：遥控模式；01：上装控制模式（用此指令时，遥控器切换按钮保证不用动）；"
    uint8_t IPC_Stop_Eme;   //上装下发的紧急停车指令 "00：正常；01:紧急停车（发送指令1时与拍下急停开关效果相同）；"
};

struct MSG_IDA2{
    uint8_t Steering_Enb;   //IPC发出转向使能信号后，给出的转向信息才有效 "00：人工接管模式；01：自动驾驶模式；"(协议文档写的人工接管和自动驾驶，其实就是转向使能)
    int16_t Steering_Pos_Req; //前轮转向角度请求 "-28度~28度"
    uint8_t Steeringmode;   //Steeringmode "01：NaN；02：NaN；03：按转角转向"
};


struct MSG_IDA3{
    uint8_t Drive_Enb;  //IPC发出驱动使能信号后，给出的驱动信息才有效
    uint16_t Drive_Tq_Req;   //驱动力矩请求 "0~2200"
    // uint8_t Reserved;   //预留位
};

struct MSG_IDA4{
    uint8_t Brake_Enb;  //IPC发出制动使能信号后，给出的制动信息才有效 "00：人工接管模式；01：自动驾驶模式；"(协议文档写的人工接管和自动驾驶，其实就是制动使能)
    uint16_t Brake_Tq_Req;   //制动力矩请求 "0~300"
    // uint8_t Reserved;   //预留位
};


//--------------------------------------控制报文封装函数声明----------------------------------------------------

bool frame_encapsulation_IDA1(const struct MSG_IDA1& msg, struct can_frame& frame);
bool frame_encapsulation_IDA2(const struct MSG_IDA2& msg, struct can_frame& frame);
bool frame_encapsulation_IDA3(const struct MSG_IDA3& msg, struct can_frame& frame);
bool frame_encapsulation_IDA4(const struct MSG_IDA4& msg, struct can_frame& frame);





//--------------------------------------状态报文----------------------------------------------------

struct MSG_IDC1{
    float Vehicle_Spd;  //整车当前车速 "0~200km/h"
    uint8_t Gear_Pos;   //挡位信号 "01：驻车；02：空挡；03：前进；04：后退；05：无效；"
    uint8_t Torque_fbk; //当前力矩 "0~50 Nm"
    uint8_t CheckSum_Byte;  //校验 "0~255"
};

struct MSG_IDC2{
    uint8_t SOC;    //电池当前SOC "0~100％"
    float Bat_Vol;  //电池电压 "0~55V"
    uint8_t Bat_Discharge_Cur;  //电池放电电流 "0~120A"
    uint8_t CheckSum_Byte;  //校验 "0~255"
};

struct MSG_IDC3{
    uint8_t Gear_Enb_fbk;   //挡位使能反馈 "00：未使能；01：使能；"
    uint8_t Steering_Enb_fbk;   //转向使能反馈 "00：未使能；01：使能；"
    uint8_t Drive_Enb_fbk;  //驱动使能反馈 "00：未使能；01：使能；"
    uint8_t Brake_Enb_fbk;  //制动使能反馈 "00：未使能；01：使能；"
    uint8_t Prk_Enb_fbk;    //驻车使能反馈 "00：未使能；01：使能；"
    uint8_t RC_Takeover_flg;    //接管标志位 "00：人工接管模式；01：自动驾驶模式；（遥控器接管标志位，此值为0时，其他接管无效，此值为1时，其他接管才有效）"
    uint8_t CheckSum_Byte;  //校验 "0~255"
};

struct MSG_IDC4{
    uint8_t Fr_Spd_Valid;   //左车轮车速有效位 "00：左轮车速无效；01：左车轮车速有效；"
    float Fr_Wheel_Spd; //左车轮轮速 "0~150 r/min"
    uint8_t Rr_Spd_Valid;   //右车轮车速有效位 "00：右轮车速无效；01：右车轮车速有效；"
    float Rr_Wheel_spd; //右车轮轮速 "0~150 r/min"
};

struct MSG_IDC5{
    uint8_t Left_Motor_spd; //左电机转速 "0~3000 r/min"
    uint8_t Rcontrol_Stop_Emcy; //遥控器紧急停车标志位 "00：正常；01：为紧急停机；"
    uint8_t Right_Motor_spd;    //右电机转速 "0~3000 r/min"
};

struct MSG_IDE1{
    uint8_t Error_code; //故障码 "01：电池箱报警；02：电机控制器报警 "
    uint8_t Code_Num;   //具体故障码 "01：单体过压或者欠压；02：电池充放电电流异常（放电大于120A，充电大于60A）；03：电池电压报警（小于44V或大于55V）；04：电池温度报警（小于-20度或大于50度）；05：电池soc过低（soc小于10%）；06：左电机控制器故障；07：右电机控制器报警；"
    uint8_t CheckSum_Byte;  //校验 "0~255"
};

//车辆车身及灯光反馈
struct MSG_ID536 {
    uint8_t VCU_VehicleLeftLampFb;  //左转向灯状态反馈 "0:off / 1:on"
    uint8_t VCU_VehicleRightLampFb; //右转向灯状态反馈 "0:off / 1:on"
    uint8_t VCU_VehicleHazardWarLampFb; //危险警示灯开关状态 "0:off / 1:on"
};

void frame_parsing_IDC1(const unsigned char* frame_data, struct MSG_IDC1& msg);
void frame_parsing_IDC2(const unsigned char* frame_data, struct MSG_IDC2& msg);
void frame_parsing_IDC3(const unsigned char* frame_data, struct MSG_IDC3& msg);
void frame_parsing_IDC4(const unsigned char* frame_data, struct MSG_IDC4& msg);
void frame_parsing_IDC5(const unsigned char* frame_data, struct MSG_IDC5& msg);
void frame_parsing_IDE1(const unsigned char* frame_data, struct MSG_IDE1& msg);

//-----------------------------------------------------------------------------------
//-->>---------------------------帧消息解析---------------------------------------<<--
//-----------------------------------------------------------------------------------
enum CanFrameID_1 {
    IDA1 = 0xA1, IDA2 = 0xA2,  //控制报文ID
    IDA3 = 0xA3, IDA4 = 0xA4
};

enum CanFrameID_2 {
    IDC1 = 0xC1, IDC2 = 0xC2,  //状态报文ID
    IDC3 = 0xC3, IDC4 = 0xC4,
    IDC5 = 0xC5, IDE1 = 0xE1
};


#endif // CAN_PROTOCOL_H

