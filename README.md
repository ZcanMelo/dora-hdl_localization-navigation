# dora-hdl_localization-navigation

基于 [dora-rs](https://github.com/dora-rs/dora) 数据流框架构建的一套激光雷达自动驾驶/导航栈。
整个系统由多个独立的 dora 节点（node）组成，通过数据流管道串联，完成从
**传感器采集 → 定位 → 地图/道路发布 → 任务规划 → 路径规划 → 底盘控制 → 可视化** 的完整流程。

---

## 系统架构

```
 ┌──────────┐        ┌──────────────────┐
 │  imu     │─imu───▶│                  │
 │ (HWT9053)│  msg   │ hdl_localization │──cur_pose─┐
 └──────────┘        │  (NDT 点云配准)  │           │
 ┌──────────┐        │                  │           │
 │  lidar   │─point─▶│                  │           │
 │(RSLidar) │  cloud └──────────────────┘           │
 └────┬─────┘                                        ▼
      │            ┌──────────┐   road_lane  ┌───────────────────────┐
      │            │ pub_road │─────────────▶│ road_lane_publisher   │
      │            └──────────┘              │  (道路+位姿融合)      │──cur_pose_all─┐
      │                                      └───────────────────────┘               │
      │            ┌──────────────┐  road_attri_msg                                   ▼
      │            │ task_pub     │──────────────────────────────────▶ ┌──────────────────┐
      │            │ (任务发布)   │                                     │    planning      │
      │            └──────────────┘                                     │ (routing/frenet) │
      │                                                                 └───────┬──────────┘
      │                                                          raw_path / Request
      │                                                                         │
      │                                          ┌──────────────────────────────┴────┐
      │                                          ▼ (下方控制节点默认注释关闭)          │
      │                              ┌───────────────┐  ┌───────────────┐  ┌──────────────────┐
      │                              │ lat_controller│  │ lon_controller│  │ vehicle_chassis  │
      │                              │ (纯跟踪横向) │  │  (纵向控制)   │  │  (N3 CAN 底盘)   │
      │                              └───────────────┘  └───────────────┘  └──────────────────┘
      │
      └───pointcloud──▶ ┌────────┐ ◀──raw_path── planning
                        │ rerun  │ ◀──cur_pose── hdl_localization
                        │ (可视) │
                        └────────┘
```

## 节点说明

| 节点 | 目录 | 说明 |
|------|------|------|
| `imu` | `HWT9053modbus/` | 维特智能 HWT9053 IMU 驱动，通过串口/Modbus 读取加速度、角速度、姿态角，输出 `imu_msg`。 |
| `lidar` | `rslidar_driver/` | 速腾聚创（RoboSense）激光雷达驱动，基于 `rs_driver`。支持在线雷达和 PCAP 回放两种模式，输出 `pointcloud`。 |
| `hdl_localization` | `dora-hdl_localization/` | 基于 NDT（`hdl_ndt_omp`）点云配准 + UKF 的地图匹配定位，融合 IMU，输出当前位姿 `cur_pose`。 |
| `pub_road` | `map/pub_road/` | 发布道路车道线信息 `road_lane`。 |
| `road_lane_publisher_node` | `map/road_line_publisher/` | 融合道路信息与定位位姿，输出全局位姿 `cur_pose_all`（含 frenet 坐标）。 |
| `task_pub_node` | `planning/mission_planning/task_pub/` | 任务/道路属性发布，输出 `road_attri_msg`（限速、AEB、停车等）。 |
| `planning` | `planning/routing_planning/` | 路径规划核心，基于 frenet 坐标系做轨迹规划，输出 `raw_path` 和控制请求 `Request`。 |
| `lat_controller` | `control/vehicle_control/lat_controller/` | 横向控制（纯跟踪 Pure Pursuit），输出 `SteeringCmd`。*默认注释关闭。* |
| `lon_controller` | `control/vehicle_control/lon_controller/` | 纵向控制，输出扭矩/制动 `TrqBreCmd`。*默认注释关闭。* |
| `vehicle_chassis_node` | `control/vehicle_control/vehicle_chassis_n3/` | N3 底盘接口，通过 SocketCAN 下发控制指令。*默认注释关闭。* |
| `rerun` | `rerun/` | 基于 [Rerun](https://rerun.io) 的可视化，实时显示点云、位姿和规划轨迹。 |

## 目录结构

```
.
├── CMakeLists.txt          # 顶层构建入口，聚合所有子节点
├── run.yml                 # dora 主数据流描述（完整管道）
├── load_path.yml           # 仅定位+传感器的精简数据流（离线调试用）
├── once_orin.sh            # Orin 平台一次性初始化（配置 CAN 总线、devmem）
├── include/                # 各节点共享的消息头（Pose、Twist、Object、Route 等）
├── HWT9053modbus/          # IMU 驱动节点
├── rslidar_driver/         # 激光雷达驱动节点（含 rs_driver 第三方库）
├── dora-hdl_localization/  # NDT 定位节点（含 hdl_ndt_omp 第三方库）
├── map/                    # 道路/地图发布节点
├── planning/               # 任务规划 + 路径规划节点
├── control/                # 横向/纵向/底盘控制节点
├── rerun/                  # 可视化节点
├── data/                   # 点云地图 (.pcd)、轨迹数据
├── map/                    # 地图相关资源
└── Waypoints*.txt          # 参考路径点
```

## 依赖

- [dora-rs](https://github.com/dora-rs/dora)（数据流运行时与 `dora_node_api_c` 库）
- CMake ≥ 3.5，支持 C++11 的编译器
- [PCL](https://pointclouds.org/)（点云处理）
- [Eigen3](https://eigen.tuxfamily.org/)
- [rerun_sdk](https://rerun.io)（可视化）
- OpenMP（NDT 加速）
- [nlohmann/json](https://github.com/nlohmann/json)（消息序列化）

> 顶层 `CMakeLists.txt` 期望在项目根目录下存在 `dora/include` 与 `dora/lib/libdora_node_api_c.a`。
> 请先安装 dora 并将对应的头文件/静态库放置到该目录（或修改 `DORA_INCLUDE_DIR`、`DORA_NODE_API_LIB`）。

## 构建

```bash
# 在项目根目录
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

各节点的可执行文件会输出到 `build/<node>/` 下，与 `run.yml` 中的 `source` 路径对应。

## 运行

在目标车载平台（如 NVIDIA Orin）上首先执行一次硬件初始化（配置 CAN 总线等）：

```bash
sudo ./once_orin.sh
```

然后使用 dora 启动数据流：

```bash
# 启动 dora 协调进程
dora up

# 运行完整管道
dora start run.yml --name yewai_slam

# 或仅运行传感器 + 定位（离线/调试）
dora start load_path.yml
```

### 常用配置

- **雷达模式**（`run.yml` 中 `lidar` 节点的 `envs`）
  - `ONLINE_LIDAR`: `0` = PCAP 回放，`1` = 在线雷达
  - `PCAP_PATH`: PCAP 文件路径
  - `LIDAR_MSOP_PORT` / `LIDAR_DIFOP_PORT`: 雷达数据/设备端口
  - `LIDAR_TYPE`: 雷达型号（如 `RSHELIOS_16`）
- **是否使用 IMU**（`hdl_localization` 节点的 `envs`）
  - `use_imu`: `1` = 融合 IMU，`0` = 仅点云定位
- **地图**：定位所用的全局点云地图位于 `data/*.pcd`

> 控制相关节点（`lat_control` / `lon_control` / `control`）在 `run.yml` 中默认被注释，
> 需要接入真实底盘时取消注释并确保 CAN 总线已初始化。

## 备注

- 本仓库将多个原本独立的 ROS/Autoware 风格模块移植/整合到 dora 数据流框架下运行。
- `Waypoints.txt` / `Waypoints_lidar.txt` / `road_msg.txt` 为参考路径与道路属性示例数据。
