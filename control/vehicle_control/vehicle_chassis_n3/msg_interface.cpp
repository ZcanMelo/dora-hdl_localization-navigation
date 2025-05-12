extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include "msg_interface.h"
#include "v_chassis_interface.h"

#include <chrono>
#include <thread>
#include <ctime>
#include <sys/time.h>
using namespace std;




void veh_pose_callback(char *imu_data)
{
    struct RtkImuState temp;
    NaviData_h *navi_data = reinterpret_cast<NaviData_h *>(imu_data);
    temp.pitch = navi_data->pitch;
    temp.roll = navi_data->roll;
    temp.heading = navi_data->heading;
    temp.speed2d = navi_data->speed2d;
    set_rtk_imu_state(temp);
    return;
}

void steer_cmd_callback(char *steer_data)
{
    SteeringCmd_h *steering_data = reinterpret_cast<SteeringCmd_h *>(steer_data);
    // struct timeval tv;
    // gettimeofday(&tv, NULL);
    // cout << "The receive_angle_time mil time is: " << tv.tv_sec << "," << tv.tv_usec/1000<<" ms " 
    // << " SteeringAngle: " << steering_data->SteeringAngle 
    // << endl;
    set_steering_cmd(steering_data->SteeringAngle);
    return;
}

void trq_bre_cmd_callback(char *tre_bre_data)
{
    struct Trq_Bre_Cmd temp;
    TrqBreCmd_h *trqbre_data = reinterpret_cast<TrqBreCmd_h *>(tre_bre_data);
    temp.bre_enable = trqbre_data->bre_enable;
    temp.bre_value = trqbre_data->bre_value;
    temp.trq_enable = trqbre_data->trq_enable;
    temp.trq_value = trqbre_data->trq_value_3;
    // std::cout << "temp.trq_value: " << temp.trq_value << std::endl;
    set_trq_bre_cmd(temp);
}

int run(void *dora_context)
{
    unsigned char counter = 0;

    while (true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            struct VehicleStat_h VehicleStatus_msg;
            struct VehicleStat VehicleStat;

            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            if (strncmp("SteeringCmd", data_id, 11) == 0)
            {
                // auto start = std::chrono::system_clock::now(); // 获取当前时间
                // std::time_t receive_angle_time = std::chrono::system_clock::to_time_t(start);
                // std::cout << "The receive_angle_time time is: " << std::ctime(&receive_angle_time); // 打印时间
                steer_cmd_callback(data);
            }

            else if (strncmp("TrqBreCmd", data_id , 9) == 0)
            {
                trq_bre_cmd_callback(data);
            }
            // else if (strcmp("navi_msg", data_id) == 0)
            // {
            //     veh_pose_callback(data);
            // }

            ///--------------------------------------------------------------------------------------------------------------------------
          


            // const int rate = 200;   // 设定频率为200HZ
            // const chrono::milliseconds interval((int)(1000/rate));

            // /// -------------------------------------------------------------------------------------------------------------------------

            // //-->>获取车辆参数
            // // get_veh_status(VehicleStat);
            // //-->>发布车辆状态
            // VehicleStatus_msg.veh_speed = 1/*VehicleStat.VehicleSpeed*/; // 汽车速度
            // struct timeval tv;
            // gettimeofday(&tv, NULL);
            // get_veh_status(VehicleStat);
            // //cout << "veh_control/msg_interface: ms" << tv.tv_usec / 1000
            //  //   << " VehicleStat.VehicleSpeed: " << VehicleStat.VehicleSpeed << endl;
  

            // VehicleStat_h* Vehicleptr = &VehicleStatus_msg;
            // char *output_data = (char *)Vehicleptr;
            // size_t output_data_len = sizeof(VehicleStatus_msg);


            // // std::string out_id = "VehicleStat";
            // // int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);

            // // if (result != 0)
            // // {
            // //     std::cerr << "failed to send output" << std::endl;
            // //     return -1;
            // // }

            // this_thread::sleep_for(interval);

                


        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "HELLO control" << std::endl;

    //----->>初始化CAN卡
    canid_t filter_canid[] = {IDA1, IDA2, IDA3, IDA4, IDC1, IDC2, IDC3, IDC4, IDC5, IDE1};
    //-->>条件编译
    #ifdef NO_LOCAL_DEBUG
        can_fd = socketcan_init("can0", LOOPBACK_RECV_OWN::LOOPBACK_RECV_OWN_OFF_OFF); // 套接字
        if (can_fd < 0) {
            perror("Failed to initialize CAN socket");
            return -1;
        }

        int filter_result = socketcan_filter_set(can_fd, filter_canid, sizeof(filter_canid) / sizeof(canid_t)); // 帧过滤
        if (filter_result < 0) {
            perror("Failed to set CAN filter");
            return -1;
        }
    #endif

    //---->>开启获取车辆状态线程 开启命令发送定时器
    pthread_t client_thread_id = start_status_monitor_thread();
    start_send_timer(10);


    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);
    //->取消线程    ->关闭CAN卡
    pthread_cancel(client_thread_id);
    close(can_fd);

    std::cout << "GOODBYE  control" << std::endl;

    return ret;
}

