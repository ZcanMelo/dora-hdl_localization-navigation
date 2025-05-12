#include "v_chassis_interface.h"
#include <iostream>
#include <cmath>
#include <complex>
#include <chrono>
#include <ctime>
#include <sys/time.h>
using namespace std;

///-->>全局变量----------------------------------------------------------------------------------------------------
int can_fd = 0;                                                              //canbus 文件描述符
///-->>静态全局变量-------------------------------------------------------------------------------------------------
// static bool   self_drive_mode = false;                                       //自动驾驶模式

struct RtkImuState rtk_state = {0, 0, 0, 0};                         //rtk状态初始化
struct Trq_Bre_Cmd trq_bre_cmd = {0, 0, 0, 0};
struct Control_Mode control_mode = {0, 0, 0, 0, 0, 0};
uint8_t turn_light = 0;

///--控制消息>>---------------------------------------------------------------------------------------------------
static struct MSG_IDA1 msg_ida1 = {0,0,0,0};
static struct MSG_IDA2 msg_ida2 = {0,0,0};
static struct MSG_IDA3 msg_ida3 = {0,0};
static struct MSG_IDA4 msg_ida4 = {0,0};

///--报文解析>>---------------------------------------------------------------------------------------------------
static struct MSG_IDC1 msg_idc1 = {0,0,0,0};
static struct MSG_IDC2 msg_idc2 = {0,0,0,0};
static struct MSG_IDC3 msg_idc3 = {0,0,0,0,0,0,0};
static struct MSG_IDC4 msg_idc4 = {0,0,0,0};
static struct MSG_IDC5 msg_idc5 = {0,0,0};
static struct MSG_IDE1 msg_ide1 = {0,0,0};

void vehicle_ready() {
    // msg_ida1.IPC_ModeCtrl = 1;
}

void set_rtk_imu_state(struct RtkImuState &arg)
{
    rtk_state = arg;
    return;
}

void set_control_mode(struct Control_Mode &arg)
{
    control_mode = arg;
    return;
}

//获取方向角度值
void set_steering_cmd(const float angle){

    msg_ida2.Steering_Enb = 1;  //转向使能
    msg_ida2.Steeringmode = 3;  //转向模式：3为按转向角转向
    msg_ida2.Steering_Pos_Req = -angle;  //转向角
 
    
    return ;
}
///-------------------------------------------------------------------------------------------------------------
void set_trq_bre_cmd(struct Trq_Bre_Cmd &msg) {
    trq_bre_cmd = msg;
    if(trq_bre_cmd.bre_enable == 1) {
        msg_ida3.Drive_Enb = 0; //关闭驱动使能
        msg_ida3.Drive_Tq_Req = 0x00;   //驱动力拒清零
        msg_ida4.Brake_Enb = 1; //打开制动使能
        msg_ida4.Brake_Tq_Req = 20;    //制动力矩赋值
    }else {
        //放手刹
        msg_ida1.Gear_Enb = 1;  //打开档位使能
        msg_ida4.Brake_Enb = 0; //关闭制动使能
        msg_ida3.Drive_Enb = 1; //打开驱动使能
        if(trq_bre_cmd.trq_value >= 0) { //前进
            msg_ida3.Drive_Tq_Req = trq_bre_cmd.trq_value;
            // std::cout << "trq_bre_cmd.trq_value: " << std::endl;
            msg_ida1.Gear_Shift_Req = 3;    //前进挡
        }else { //后退
            msg_ida3.Drive_Tq_Req = trq_bre_cmd.trq_value;
        }
    }    
}

static void *status_monitor_thread_recall(void *arg);

/**
 * @brief manual_auto_driving_thread_recall
 * @param arg
 * @return
 */

static void msg_send_recall(int signo);

//-->>以下方法为软件框架，不需要优化
/**
 * @brief start_status_monitor_thread 开启车辆状态监控线程
 * @return
 */
pthread_t start_status_monitor_thread()
{
    pthread_t id = 0;
    // 开启获取车辆状态线程
    if (pthread_create(&id, nullptr, status_monitor_thread_recall, nullptr) != 0)
    {
        std::cerr << "create getCarInforThread thread fail!" << std::endl;
        exit(-1);
    }
    return id;
}

/**
 * @brief start_send_timer开启控制报文发送定时器
 * @param t_ms
 */
void start_send_timer(int t_ms)
{
    signal(SIGALRM, msg_send_recall); // SIGALRM是信号类型，收到就执行后面的函数
    struct itimerval it, oldit;
    it.it_value.tv_sec = 0;               // 第一次执行时间初始时间
    it.it_value.tv_usec = t_ms * 1000;    // 20ms
    it.it_interval.tv_sec = 0;            // 间隔时间
    it.it_interval.tv_usec = t_ms * 1000; // 20ms

    if (-1 == setitimer(ITIMER_REAL, &it, &oldit)) //
    {
        perror("msg_send_timer_init failed");
        exit(-1);
    }
    return;
}

/**
 * @brief msg_send_recall
 * @param signo
 */
static void msg_send_recall(int signo)
{
    if(signo != SIGALRM) return;
    // vehicle_ready();
    static struct can_frame frame;                    //发送帧结构
    //--------------------------------------------------------------------------------
    //-->>发送控制消息
    if(msg_ida3.Drive_Enb == 1 && msg_ida4.Brake_Enb == 1)
        msg_ida3.Drive_Enb = 0;

    memset(&frame,0,sizeof (frame));//清空缓存区
    msg_ida1.Gear_Enb = 1;
    msg_ida1.Gear_Shift_Req = 3;
    msg_ida1.IPC_ModeCtrl = 0;
    msg_ida1.IPC_Stop_Eme = 0;
                                
    frame_encapsulation_IDA1(msg_ida1,frame);        //填充帧消息
    //-->>条件编译
    write_socketcan_frame(can_fd,frame);             //发送帧消息


    memset(&frame, 0, sizeof(frame)); 
    // std::cout << "IDA2" << std::endl;                                 
    frame_encapsulation_IDA2(msg_ida2,frame);        //填充帧消息
    write_socketcan_frame(can_fd,frame);              //发送帧消息


    memset(&frame, 0, sizeof(frame)); 
    // msg_ida3 = {1, 25};
    // msg_ida3.Drive_Enb = 1;
    // msg_ida3.Drive_Tq_Req = 80;
    // std::cout << "msg_ida3.Drive_Tq_Req: " << msg_ida3.Drive_Tq_Req << std::endl;                                 
    frame_encapsulation_IDA3(msg_ida3,frame);        //填充帧消息
    write_socketcan_frame(can_fd,frame);           //发送帧消息


    memset(&frame, 0, sizeof(frame)); 
    frame_encapsulation_IDA4(msg_ida4,frame);        //填充帧消息
    // std::cout << "IDA4" << std::endl;                                 
    write_socketcan_frame(can_fd,frame);  
    return;
}

/**
 * @brief set_TurnLight_cmd
 * @param TurnLight
 */
// void set_TurnLight_cmd(const uint8_t TurnLight)
// {
//     turn_light = TurnLight;
//     switch (turn_light)
//     {
//     case 0:
//         mgs_id133.ACU_VehicleLeftLampCtrl = 0;
//         mgs_id133.ACU_VehicleRightLampCtrl = 0;
//         break;
//     case 1:
//         mgs_id133.ACU_VehicleLeftLampCtrl = 1;
//         mgs_id133.ACU_VehicleRightLampCtrl = 0;
//         break;
//     case 2:
//         mgs_id133.ACU_VehicleLeftLampCtrl = 0;
//         mgs_id133.ACU_VehicleRightLampCtrl = 1;
//         break;
//     case 3:
//         mgs_id133.ACU_VehicleLeftLampCtrl = 1;
//         mgs_id133.ACU_VehicleRightLampCtrl = 1;
//         break;
//     }
//     return;
// }

/**
 * @brief get_veh_status
 * @param stutus
 */
void get_veh_status(struct VehicleStat &stutus)
{
    // stutus.VehicleSpeed = rtk_state.speed2d;            // mgs_id193.speed_feedback;
    // stutus.VehicleSpeed = mgs_id530.VCU_ChassisSpeedFb; // mgs_id193.speed_feedback;

    // // ROS_INFO_THROTTLE(2, "VehicleSpeed= %f", stutus.VehicleSpeed);
    return;
}

/**
 * @brief status_monitor_thread_recall
 * @param arg
 * @return
 */
void* status_monitor_thread_recall(void* arg) {
struct can_frame frame;
    while(true)
    {
        memset(&frame,0,sizeof (struct can_frame));              //清空接收缓存区
        //-->>条件编译
        // #ifdef NO_LOCAL_DEBUG
        // read(can_fd,&frame,sizeof(frame));
        // #endif

        ssize_t bytesRead = read(can_fd, &frame, sizeof(frame));
        if (bytesRead != sizeof(frame)) {
            std::cerr << "Error: Expected to read " << sizeof(frame) << " bytes, but got " << bytesRead << " bytes" << std::endl;
            return nullptr;
        }

        // std::cout << "id: " << frame.can_id << std::endl;
        switch (frame.can_id)
        {
            case IDC1 :
                frame_parsing_IDC1(frame.data, msg_idc1);         //解析
                // std::cout<< "Vehicle_Spd: " << msg_idc1.Vehicle_Spd << std::endl;
                break;
            case IDC2 : 
                frame_parsing_IDC2(frame.data, msg_idc2);
                break;
            case IDC3 :
                frame_parsing_IDC3(frame.data, msg_idc3);
                break;
            case IDC4 :
                frame_parsing_IDC4(frame.data, msg_idc4);
                break;
            case IDC5 :
                frame_parsing_IDC5(frame.data, msg_idc5);
                break;
            case IDE1 :
                frame_parsing_IDE1(frame.data, msg_ide1);
                break;
            default: break;
        }
    }
    return arg;
}
