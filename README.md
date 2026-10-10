# Adam_nav_2027

面向 ASUS NUC 15 Pro、Ubuntu 24.04、ROS 2 Jazzy 的 RoboMaster 全向哨兵导航框架。
架构依据 `RM_Nav_V2` 技术方案 v2.5，通信复用此前实车运行的 `my_serial_py`。
构建与运行验证在 `asus@192.168.1.145` 完成，代码同步至
[kswlt/Adam_nav_2027](https://github.com/kswlt/Adam_nav_2027)。

## 目录

- [项目定位与边界](#项目定位与边界)
- [五分钟快速开始](#五分钟快速开始)
- [运行模式与启动顺序](#运行模式与启动顺序)
- [框架架构](#框架架构)
- [数据流、TF 与接口](#数据流tf-与接口)
- [实机使用：MID-360、建图和定位](#实机使用mid-360建图和定位)
- [Foxglove 可视化](#foxglove-可视化)
- [构建、测试与结果判读](#构建测试与结果判读)
- [故障排查](#故障排查)
- [当前进度与未完成边界](#当前进度与未完成边界)
- [阶段 B 硬件协议契约](#阶段-b-硬件协议契约)

## 项目定位与边界

本仓库是 RM Nav V2 的可运行 ROS 2 Jazzy 框架，目标平台是 ASUS NUC 15 Pro，目标底盘是 RoboMaster 全向哨兵。它把原版实车工程中的 `my_serial_py` 通信、MID-360/Point-LIO、动态云台 TF、状态估计、建图、重定位、Nav2 全向基线和监管逻辑组织成可分别验收的模块。

代码按“先能观测、再能定位、再能建图、最后才允许规划输出”的顺序工作。默认启动不会把 `/nav/cmd_vel_safe` 接到物理串口，也不会用静态 `map → odom` 冒充生产定位。测试替身、原版静态外参和理想底盘反馈只能用于对应的测试 profile。

### 这套代码适合做什么

1. 在独立 ROS Domain 中验证消息、TF、健康状态、Costmap、MPPI、串口协议和恢复事务。
2. 接入真实 MID-360，查看原始点云、质量门、Point-LIO、局部子地图和轨迹。
3. 记录带标定 ID 的原始关键帧，离线运行 VGICP/GTSAM 建图，并用 Foxglove 检查结果。
4. 在完成实测标定、轮速/停车反馈和硬件验收后，逐步接入真实底盘。

### 当前明确不做的事情

- 不使用 `standard_robot_pp_ros2`；串口入口始终是 `my_serial_py`。
- 不伪造编码器、轮速、停车反馈、真值、实测外参或正式地图。
- 不在未完成硬件验收前发送物理底盘运动指令。
- 不把 `mapping_odom` 下的离线 PCD 直接当成正式 `map`。

## 五分钟快速开始

以下命令在 ASUS 上执行。每个新终端都要重新 source；不要先 source 本工程旧的 `install` 再编译。

```bash
cd /home/asus/nav_2027/rm_nav_v2
source /opt/ros/jazzy/setup.bash
source /home/asus/nav_deps/install_lio/setup.bash
MAKEFLAGS=-j2 CMAKE_PREFIX_PATH=/home/asus/nav_deps/install:${CMAKE_PREFIX_PATH:-} \
  colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

先跑一个不会接触真实硬件的全向导航基线：

```bash
ROS_DOMAIN_ID=87 python3 tools/smoke_mppi_baseline.py
```

再跑完整回归：

```bash
colcon test
colcon test-result --verbose
```

看到 `0 errors, 0 failures` 才能把该环境用于下一步调试。测试中出现 `skipped` 时，要查看具体原因；当前可选的原版 `libscrc` 兼容检查属于已知跳过项。

## 运行模式与启动顺序

### A. 纯软件测试模式

软件测试不需要 MID-360、STM32 或 Foxglove。推荐每个测试使用不同 `ROS_DOMAIN_ID`，避免旧节点互相订阅：

```bash
ROS_DOMAIN_ID=87 python3 tools/smoke_mppi_baseline.py
ROS_DOMAIN_ID=88 python3 tools/smoke_serial_pty.py
ROS_DOMAIN_ID=89 python3 tools/smoke_frozen_map_localization.py
ROS_DOMAIN_ID=90 python3 tools/smoke_motion_gate.py
ROS_DOMAIN_ID=91 python3 tools/smoke_kiss_recovery.py
ROS_DOMAIN_ID=102 python3 tools/smoke_mapping_capture.py
python3 tools/smoke_offline_mapping.py
```

这些脚本会创建合成输入或理想反馈，用来验证软件契约，不代表实车精度和制动性能。

### B. MID-360 实机观测模式

先确认 ASUS 网卡和雷达路由正常。当前约定是 Wi-Fi `192.168.1.145` 用于 SSH/外网，有线 `192.168.1.50/32` 只访问雷达 `192.168.1.3`。有线网卡不要改回与 Wi-Fi 重叠的 `/24`。

终端 1，启动官方 Livox 驱动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/asus/nav_deps/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

终端 2，先做诊断或启动状态链。生产模式使用实测 CalibrationBundle：

```bash
source /opt/ros/jazzy/setup.bash
source /home/asus/nav_deps/install_lio/setup.bash
source /home/asus/nav_2027/rm_nav_v2/install/setup.bash
ros2 launch rm_nav_bringup local_state.launch.py \
  calibration_file:=/absolute/measured_calibration.yaml \
  lio_params_file:=/absolute/lio_params.yaml \
  enable_lio:=true enable_mid360_guard:=true
```

如果只是复现原版静态外参联调，必须显式标记为 legacy：

```bash
ros2 launch rm_nav_bringup local_state.launch.py \
  calibration_file:=/home/asus/nav_2027/rm_nav_v2/src/rm_nav_frames/config/local_state_calibration.legacy_adam_static.yaml \
  lio_params_file:=/home/asus/nav_deps/src/small_point_lio/config/mid360.yaml \
  enable_lio:=false enable_mid360_guard:=false \
  allow_legacy_static_calibration:=true
```

legacy 模式只用于接口检查，不能用于宣布实测定位通过。

### C. 建图和导航模式

启动顺序固定为：传感器 → LIO/状态 → 观测子地图或关键帧 → Foxglove → Nav2。Nav2 需要 `/map`、`/odom`、`/scan` 和完整 TF，因此不能只启动 Nav2 就期待出现地图或点云。

实机关键帧采集：

```bash
ros2 launch rm_nav_bringup mapping_capture.launch.py \
  archive_root:=/home/asus/nav_data/mapping_sessions \
  calibration_id:=measured-bundle-v1 \
  body_frame:=base_footprint sensor_frame:=front_mid360
```

每次采集使用新的 session。当前单个归档最多 500 帧；达到上限会停止并报告，需要新建分段，不能静默丢帧。停止采集后离线优化：

```bash
ros2 run rm_nav_mapping offline_graph_optimizer \
  /home/asus/nav_data/mapping_sessions/<session> \
  /home/asus/nav_data/mapping_outputs/<output>
```

输出坐标是 `mapping_odom`。完成官方场地对齐、地图清理、占据栅格和 MapBundle 发布前，不要把它加载为正式冻结地图。

MPPI 全向基线：

```bash
ros2 launch rm_nav_bringup mppi_baseline.launch.py \
  enable_task_supervisor:=true \
  enable_pointcloud_scan_adapter:=true
```

`enable_pointcloud_scan_adapter` 只有在传感器高度、TF 和 footprint 已实测后才打开。导航任务通过 `/nav/submit_goal` 交给 task supervisor；直接向 Nav2 action 发送目标不会自动获得运动许可。当前 `/nav/cmd_vel_safe` 仍与物理串口隔离。

## 框架架构

```text
MID-360/Livox ──> cloud guard ──> Point-LIO ──> sensor-pose resolver
       │                                  │                 │
       │                                  └─> odom submap ──┤
       │                                                    v
       └──────────── Foxglove/raw topics              EKF / state bridge
                                                            │
                                      odom→base_footprint ──┘
                                                            │
             my_serial_py <── Hardware boundary       Nav2 / MPPI
                 │                                          │
                 └── chassis/gimbal feedback          safe command gate

mapping capture ──> VGICP/GTSAM ──> mapping_odom PCD ──> MapBundle (待正式对齐)
frozen map + odom submap ──> KISS/GICP ──> MapOdomManager ──> map→odom
```

包的职责边界如下：

| 层 | ROS 包 | 主要职责 |
| --- | --- | --- |
| 硬件与接口 | `my_serial_py`, `pb_rm_interfaces`, `rm_nav_hardware` | 复用原协议、串口桥、硬件健康和反馈边界 |
| Frame 与状态 | `rm_nav_frames`, `rm_nav_localization`, `rm_nav_sensors` | CalibrationBundle、云台 TF、Point-LIO 观测、resolver、EKF 和状态桥 |
| 配准与定位 | `rm_nav_registration`, `rm_nav_localization` | small_gicp、KISS/GICP、冻结地图定位、恢复事务 |
| 建图 | `rm_nav_mapping` | 关键帧归档、VGICP 相邻边、GTSAM、回环候选、地图清单 |
| 感知与规划 | `rm_nav_perception`, `rm_nav_planning`, `rm_nav_control` | 点云/Costmap 适配、路径、轨迹、全向控制接口和 yaw 管理 |
| 启动与监管 | `rm_nav_bringup`, `rm_nav_sim` | launch、任务监管、故障门、仿真和集成 smoke |

## 数据流、TF 与接口

### TF 所有权

```text
map ──(MapOdomManager 唯一发布)──> odom
odom ──(robot_localization 唯一发布)──> base_footprint
base_footprint ──(标定)──> chassis
chassis ──(绝对编码器/云台状态)──> big_gimbal_yaw
big_gimbal_yaw ──(CalibrationBundle)──> front_mid360
```

`map → odom` 只允许重定位管理器修改；重定位不能跳变 `odom → base_footprint`。`front_mid360` 是传感器 frame，不能直接当作 `base_link`。`base_link` 的定义必须与 Nav2 参数一致，不能同时由两个节点发布。

### 关键话题

| 方向 | 话题 | 说明 |
| --- | --- | --- |
| 原始输入 | `/livox/lidar`, `/livox/imu` | Livox 原始点云和 IMU；保留源时间，不能重复加 header 时间 |
| 质量门 | `/sensors/front_mid360/guarded_points`, `/sensors/front_mid360/cloud_healthy`, `/sensors/front_mid360/cloud_reason` | 质量通过的原始 payload、健康状态和拒绝原因 |
| LIO | `/lio/sensor_odometry`, `/lio/deskewed_odom_cloud` | 原始 LIO 位姿和已经在 odom 中的去畸变点云 |
| 状态 | `/state/chassis`, `/state/chassis_healthy`, `/odom` | 底盘状态、健康心跳和 Nav2 参考点里程计 |
| 建图 | `/localization/odom_submap`, `/mapping/keyframe_archive` | 局部观测子地图、关键帧提交状态 |
| 定位 | `/localization/frozen_map`, `/localization/estimate`, `/localization/healthy` | 冻结地图输入、定位估计和健康状态 |
| 导航 | `/scan`, `/global_costmap/costmap`, `/local_costmap/costmap` | Costmap 输入和结果 |
| 命令 | `/nav/cmd_vel_raw`, `/nav/cmd_vel_smoothed`, `/nav/cmd_vel_checked`, `/nav/cmd_vel_safe` | Nav2 到安全输出门的速度链，当前不直连物理底盘 |
| 原串口 | `/cmd_vel`, `/cmd_yaw_angle` | `my_serial_py` 的 xy 平移和 yaw 目标角；协议不发送 `angular.z` |

### 健康与停车逻辑

安全输出需要新鲜的 `/state/chassis_healthy`、`/localization/healthy`、`/nav/motion_enable` 和有效速度。点云质量门、LIO、定位、任务监管或串口 watchdog 任一失效，输出门应回到零速度。EKF 估计速度不能当作硬件实测停车反馈。

## 实机使用：MID-360、建图和定位

### 采集建议

静止检查保持雷达稳定 60 秒；动态检查分别采集 1 m 直线、横移、原地旋转和往返闭环。记录场地、速度、云台角度、遮挡情况和开始/结束时间。没有外部真值时，只报告链路、时间、健康、漂移和资源指标，不把估计轨迹称为定位真值。

### 正式冻结地图前的门槛

`create_map_bundle.py` 生成的清单默认是 `draft_mapping_odom`。只有完成真实 PCD 验证、输入一致性检查、官方坐标对齐、动态点清理、occupancy/terrain 生成和人工验收后，才允许进入 `frozen_map_localization.launch.py`。draft Bundle 会被启动器拒绝。

## Foxglove 可视化

启动 Bridge：

```bash
ros2 launch rm_nav_bringup foxglove_visualization.launch.py
```

Windows Foxglove 连接 `ws://192.168.1.145:8765`。3D 面板固定 frame：看实时 LIO/子地图用 `odom`，看正式地图和全局 Costmap 用 `map`，看离线优化结果用 `mapping_odom`。建议同一时间只显示一个高带宽点云：原始 `/livox/lidar`、质量门点云和实时累积地图同时显示会造成明显延迟。

最小可用布局：

| 面板 | 添加内容 |
| --- | --- |
| 3D | TF、`/visualization/live_map_preview`、`/localization/odom_submap`、一条点云、三条 Path |
| Raw Messages | `/state/chassis_healthy`、`/localization/healthy`、`/sensors/front_mid360/cloud_reason` |
| Plot | `/state/chassis` 位置/速度、`/lio/sensor_odometry`、健康状态 |
| 3D（全局） | `/visualization/map_cloud`、`/global_costmap/costmap`、`/scan`、TF |

加载离线地图和优化图：

```bash
ros2 launch rm_nav_bringup foxglove_visualization.launch.py \
  map_pcd:=/absolute/rebuilt_map.pcd \
  map_frame:=mapping_odom \
  graph_poses:=/absolute/optimized_poses.json \
  graph_loops:=/absolute/loop_edges.json \
  map_bundle:=/absolute/map_bundle.json
```

实时建图时，Bridge 只负责显示，不会自动启动驱动、LIO 或关键帧采集；必须先启动传感器和状态链。看到话题但 3D 空白时，先检查固定 frame 是否能通过 TF 变换到该话题的 frame，再检查话题是否真的有新消息和时间戳是否在前进。

## 当前进度与未完成边界

截至 2026-10-10，工程含 14 个 ROS 包，远端全量构建通过。
最近一次 P0 全量测试汇总为 38 项、0 失败、1 跳过；跳过项是可选的原版 libscrc 兼容核验，
单独加载 libscrc 1.8.1 后全部 13 项串口协议测试通过。

| 功能 | 已验证内容 | 尚未完成 |
| --- | --- | --- |
| 原版通信 | 26-byte RX / 54-byte TX、原 CRC/符号/yaw/话题兼容、ROS 虚拟串口、平移超时停车 | 实车抓包、断线重连和下位机 watchdog 联调 |
| Nav2 基线 | Smac2D + MPPI Omni → smoother → collision monitor；横移导航、障碍/雷达断流/命令超时停车 | 实车定位输入、测量足迹、硬件速度与 yaw 适配 |
| 配准与恢复 | 真实 GICP/KISS、双候选、受限自动触发、停车/取消/TF/清图、新规划放行 | 真实点云回放与实车稳定性验证 |
| TF 与定位 | 上游 small_point_lio、时间对齐云台 TF/SE(3) resolver、真实 EKF、独占 map→odom | 实测标定、硬件同步与真实回放 |
| Sensor Hub | 主 LIO 原生观测、源原点、角色约束、有界定位子地图；MID-360 实机传输/时间字段及无遮挡静止回放通过 | 动态真实回放、多源及感知适配 |
| 优化建图 | 原始关键帧采集、真实 VGICP 相邻因子、GTSAM 位姿链与原始点云重建 | KISS 回环复核、官方对齐、地图清理与 MapBundle 发布 |
| 最终规划控制 | 时序轨迹、路径与足迹验证基础接口 | TDT 后端、Omni PID、YawManager、ros2_control 适配 |

**P0–P10 尚未全部完成。** 数据结构和单元测试不等于完整导航功能。
详细进度见 [阶段验收矩阵](docs/stage_completion_matrix.md)。
接手开发见 [交接文档](docs/HANDOFF.md)，包含部署、真实数据证据、未完成工作和重现命令。
Foxglove 查看地图、TF、原始点云、LaserScan 及实时建图请看 [Foxglove 可视化说明](docs/foxglove_visualization.md)。

MPPI 基线已提供官方 `pointcloud_to_laserscan` 适配器，可将质量门后的 MID-360 点云转换为
Nav2 使用的 `/scan`；默认关闭，必须先完成实测 TF、传感器高度和 footprint 验收后再启用，
详见 [MPPI 点云适配说明](docs/mppi_pointcloud_scan_adapter.md)。
已用原版静态 TF 完成现场接口联调，`/scan` 已收到 `base_link` 帧的 360 线束；证据仅覆盖
消息接口和 Costmap 输入，不代表实测标定或整车验收。
当前 Nav2 Costmap 联调已确认 `/scan` 输入正常，但因尚未启动真实 LIO→EKF 动态
`odom→base_link` TF，Costmap 不能激活；不能用静态 TF 绕过该前置条件。
已增加显式 legacy 联调模式，Point-LIO→resolver→EKF 可输出 `/state/chassis`
和动态 `odom→base_link`；该模式使用原版外参假设，不能替代实测标定。

MID-360 已接通 ASUS，修复有线/无线重叠路由。原始点云质量门已接入，可拒绝有效回波不足、
几何退化、时间异常和断流，并联锁 LIO 健康；移除遮挡后每帧有效点约 1.41–1.48 万，
稳定段点云 10 Hz、IMU 200 Hz；真实 rosbag 静止回放约 20.7 s，最大位置变化 8.4 mm、
最大姿态变化约 0.09°，本次静止检查通过，动态和整车验收仍待完成。
接线配置、原始 rosbag 及遮挡失败对照见 [实机接入记录](docs/mid360_hardware_acceptance.md)。

### 构建（asus 主机）

```bash
cd /home/asus/nav_2027/rm_nav_v2
source /opt/ros/jazzy/setup.bash
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_small_gicp.sh
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_kiss_matcher.sh
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_small_point_lio.sh
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_gtsam.sh
source /home/asus/nav_deps/install_lio/setup.bash
MAKEFLAGS=-j2 CMAKE_PREFIX_PATH=/home/asus/nav_deps/install:${CMAKE_PREFIX_PATH:-} colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

远端已安装 Nav2 bringup/MPPI/collision monitor/velocity smoother、robot_localization、
ros2_control 和 controllers。small_gicp、KISS-Matcher、ROBIN 在用户目录构建，精确提交见
[dependencies.repos](dependencies.repos)。[依赖清单](dependencies.lock.yaml) 目前仅部分锁定。

### 已实现的启动与验收

```bash
# 独立 Domain 中运行理想全向模型，启动真实 Nav2 节点。
ROS_DOMAIN_ID=87 python3 tools/smoke_mppi_baseline.py

# 伪终端模拟 STM32，启动真实串口 ROS 节点，不打开物理串口。
ROS_DOMAIN_ID=88 python3 tools/smoke_serial_pty.py

# 真实 GICP 算法，验证已知变换和异常数据拒绝。
./build/rm_nav_registration/small_gicp_backend_test

# 真实 KISS 粗配准与 GICP 精化，验证大初始偏差和候选门。
./build/rm_nav_registration/kiss_gicp_backend_test

# 真实配准和 TF 节点，模拟冻结地图及 odom 子地图；时间/版本/大修正拒绝。
ROS_DOMAIN_ID=89 python3 tools/smoke_frozen_map_localization.py

# 失健康、失许可、心跳中断、无缓存重放与非法命令拒绝。
ROS_DOMAIN_ID=90 python3 tools/smoke_motion_gate.py

# 真实受限 KISS/GICP ROS 恢复、独立候选确认、会话和停车输出保持。
ROS_DOMAIN_ID=91 python3 tools/smoke_kiss_recovery.py

# 真实 Nav2 取消旧目标、提交定位和清理代价地图；停车反馈为理想模型。
ROS_DOMAIN_ID=92 python3 tools/smoke_recovery_transaction.py
# 服务故障替身验证提交前/后的失败保持。
ROS_DOMAIN_ID=93 python3 tools/smoke_recovery_transaction_faults.py cancel_rejected
ROS_DOMAIN_ID=94 python3 tools/smoke_recovery_transaction_faults.py clear_timeout
ROS_DOMAIN_ID=95 python3 tools/smoke_recovery_transaction_faults.py map_changed

# 自动受限搜索与真实 Nav2 新任务/恢复许可；仍使用理想底盘反馈。
ROS_DOMAIN_ID=96 python3 tools/smoke_auto_recovery.py
ROS_DOMAIN_ID=97 python3 tools/smoke_task_resume.py

# 真实上游 Point-LIO：合成 PointCloud2/IMU 输入，不接物理传感器。
ROS_DOMAIN_ID=98 python3 tools/smoke_small_point_lio.py

# 真实云台 TF、resolver 和 EKF；编码器/原始位姿为明确的合成测试输入。
ROS_DOMAIN_ID=99 python3 tools/smoke_local_state.py

# 实际 LIO / EKF / 原生观测 / 子地图 / GICP；输入仍是合成数据。
ROS_DOMAIN_ID=100 python3 tools/smoke_small_point_lio.py --with-state
ROS_DOMAIN_ID=101 python3 tools/smoke_observation_submap.py

# 建图原始关键帧归档：故障/配额边界与实际 LIO 软件链。
ROS_DOMAIN_ID=102 python3 tools/smoke_mapping_capture.py
ROS_DOMAIN_ID=103 python3 tools/smoke_small_point_lio.py --with-state --with-mapping

# 离线归档→真实 VGICP→GTSAM→原始关键帧重建；显式合成漂移初值。
python3 tools/smoke_offline_mapping.py
```

Nav2 外部输入：`/map`、`/odom`、`/scan` 和 `map → odom → base_link` TF。
启动：`ros2 launch rm_nav_bringup mppi_baseline.launch.py`。
输出链 `/nav/cmd_vel_raw → /nav/cmd_vel_smoothed → /nav/cmd_vel_checked → /nav/cmd_vel_safe`
均为 `TwistStamped`、底盘坐标速度。当前输出隔离，尚未自动接到串口。
footprint、停止区和速度约束目前为调试初值，需要实际尺寸与制动数据。
最终输出门要求新鲜的 `/state/chassis_healthy`、`/localization/healthy` 和 `/nav/motion_enable` true 心跳，
以及新鲜有效的底盘坐标速度；默认零输出。launch 默认启动 task_supervisor，
通过 `/nav/submit_goal` 提交任务；只有监管拥有的唯一活动目标能够获得许可。
直接向 Nav2 提交目标不会自动获得运动许可。
见 [运动许可验收](docs/motion_gate_acceptance.md)。

冻结地图定位启动：
`ros2 launch rm_nav_bringup frozen_map_localization.launch.py map_version:=<实际地图版本>`。
输入 `/localization/frozen_map`（map 坐标、transient local）和
`/localization/odom_submap`（odom 坐标、源观测时间戳、已去畸变）。
输出 `/localization/estimate`、`map → odom` TF、`/localization/healthy` 和
`/localization/correction_pending`。大修正当前只保留候选；KISS 会话已支持两份不同云的
一致性复核。显式启用 `enable_recovery_transaction:=true` 后，可通过
`/localization/commit_recovery` 请求实测停车、取消旧目标、TF提交和清图事务。
它要求独立的 `/hardware/measured_twist`；原串口没有速度反馈，目前不自动提供该话题。
提交后停在 WAIT_REPLAN；稳定的提交后局部定位、新鲜规划路径与新 Nav2 目标
经验证后可进入 TRACKING，由任务监管决定运动许可。
见 [自动恢复与新任务验收](docs/task_supervisor_acceptance.md)。
见 [恢复提交事务验收](docs/recovery_transaction_acceptance.md)。
恢复默认关闭，开启时必须指定场地边界；通过 `/localization/request_recovery` 请求，
期间健康保持 false。额外开启 `enable_auto_recovery:=true` 才会根据最后可靠位置、
新鲜 odom 和观测丢失自动创建一次受限搜索。见 [受限恢复 ROS 验收](docs/kiss_recovery_ros_acceptance.md)。
详细约定见 [冻结地图定位验收](docs/frozen_map_localization_acceptance.md)。

串口启动：
`ros2 launch my_serial_py serial.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200 cmd_vel_timeout:=0.3`。
运行入口 `serial_bridge.py`；原版 `serialpy_node.py` 保留用于对照。
`/cmd_vel` 仅提供 xy 平移；`/cmd_yaw_angle` 提供 yaw 目标角（度），
报文不发送 `angular.z`。因此现有平移 watchdog 不能替代全底盘停车联调。

本地状态启动：
`ros2 launch rm_nav_bringup local_state.launch.py calibration_file:=<实测标定> lio_params_file:=<驱动/滤波配置>`。
模板未测量并默认拒绝启动；绝对云台反馈来自 `/hardware/gimbal_joint_states`，不使用
`/contact_angle`。LIO 发布原始 IMU 位姿，resolver 按源时刻查询外参并保留完整 SE(3)，
EKF 唯一发布 odom→base_footprint，状态输出 `/state/chassis`。
状态桥按完整参考点变换输出 Nav2 `/odom`，本地健康同时约束输出门与任务监管。
EKF 估计速度不等于硬件实测停车反馈，输出仍未连接物理底盘。
主 LIO 经原生观测适配与滚动窗口输出 `/localization/odom_submap`。
见 [状态桥与观测链验收](docs/state_observation_acceptance.md)。
见 [动态云台状态链验收](docs/local_state_acceptance.md)。

### 包结构

| 包 | 职责 |
| --- | --- |
| `my_serial_py` / `pb_rm_interfaces` | 原通信协议与裁判状态消息 |
| `rm_nav_hardware` / `rm_nav_frames` | 硬件边界、标定与 TF 所有权 |
| `rm_nav_interfaces` / `rm_nav_sensors` | 导航消息、观测数据契约 |
| `rm_nav_registration` / `rm_nav_localization` | 配准、LIO 底盘解算、局部定位与重定位 |
| `rm_nav_mapping` | 关键帧、回环、优化建图与地图版本管理 |
| `rm_nav_perception` | 环境、地形、窄道与代价地图适配 |
| `rm_nav_planning` / `rm_nav_control` | 全局路径、轨迹、全向控制与 yaw 管理 |
| `rm_nav_bringup` / `rm_nav_sim` | 启动配置、监管、仿真与集成验收 |

### 验收记录与后续顺序

- [MPPI 软件基线](docs/mppi_baseline_acceptance.md)
- [串口协议及 ROS/PTY](docs/serial_acceptance.md)
- [small_gicp 后端](docs/small_gicp_acceptance.md)
- [small_point_lio 上游接入](docs/small_point_lio_acceptance.md)
- [动态云台与真实 EKF](docs/local_state_acceptance.md)
- [状态桥与原生观测子地图](docs/state_observation_acceptance.md)
- [原始关键帧采集与持久化](docs/mapping_capture_acceptance.md)
- [VGICP 相邻因子与 GTSAM 离线重建](docs/offline_mapping_acceptance.md)
- [KISS→GICP 后端](docs/kiss_gicp_acceptance.md)
- [受限 KISS 恢复 ROS 链](docs/kiss_recovery_ros_acceptance.md)
- [停车与恢复提交事务](docs/recovery_transaction_acceptance.md)
- [自动恢复与新任务许可](docs/task_supervisor_acceptance.md)
- [冻结地图 ROS 定位链](docs/frozen_map_localization_acceptance.md)
- [定位健康与运动许可](docs/motion_gate_acceptance.md)
- [串口字段说明](docs/my_serial_py_interface.md)
- [架构与 TF 契约](docs/architecture_contract.md)

下一步完成原始驱动、多雷达独立观测、实测标定、硬件时间同步与真实回放；
实车速度反馈仍需独立适配。
建图记录入口为 `mapping_capture.launch.py`，显式指定存储根目录与同一标定 ID；
停止记录后可运行 `ros2 run rm_nav_mapping offline_graph_optimizer <session目录> <新输出目录>`。
已接通 VGICP/GTSAM 与原始点云重建，输出为尚未官方对齐的 mapping_odom 坐标，
不会自动进入运行时地图；仍未生成最终 MapBundle。
之后接入 KISS 回环、官方对齐与地图清理/发布，再完成 TDT 和最终控制链。
每个独立步骤先在 asus 验证，再以中文 commit 推送。提交身份统一为 `kswlt`。

框架完成后，必须自行寻找公开 rosbag，在 asus 上进行真实数据回放及系统验收，
修复问题并上传测试配置与报告。见 [rosbag 系统测试计划](docs/rosbag_system_test_plan.md)。

## 故障排查

### Foxglove 连接成功但画面空白

先执行 `ros2 topic list` 和 `ros2 topic hz <topic>`，确认当前 Domain 中确实有新消息；再在 3D 面板选择与数据一致的固定 frame。原始点云通常是 `front_mid360`，去畸变点云和局部子地图通常是 `odom`。如果固定 frame 是 `map` 或 `mapping_odom` 而没有对应 TF，Foxglove 会列出话题但无法绘制。

### Foxglove 延迟四五秒或明显卡顿

只打开一个高带宽点云。调试雷达时选 `/livox/lidar`，调试建图时选 `/visualization/live_map_preview` 或 `/localization/odom_submap`，不要同时打开原始点云、guarded 点云、去畸变点云和累积地图。Bridge 默认已经限制发送缓冲和实时预览体素；延迟仍高时先关闭 Plot/Raw Messages 中的高频数组字段，再检查 ASUS CPU 和无线链路。

### `odom → base_link` 缺失或 TF 面板出现红色错误

检查是否启动了 `local_state.launch.py`，以及 `calibration_file` 是否为真实 Bundle。生产模式由 robot_localization 发布 `odom → base_footprint`，状态桥提供 Nav2 参考点；不能启动第二个静态 `odom → base_link`。`mapping_odom` 是离线建图坐标，与实时 `odom` 没有天然 TF，离线地图要把 3D 固定 frame 设为 `mapping_odom`。

### Costmap 节点启动但不激活

Costmap 至少需要 `/scan`、地图输入和动态 `odom → base_link`。独立测试使用：

```bash
ROS_DOMAIN_ID=131 ros2 launch rm_nav_bringup costmap_mid360_smoke.launch.py \
  profile:=TEST_ONLY map_yaml:=/tmp/smoke_map.yaml
```

`TEST_ONLY` 仅创建测试用静态 TF；`LEGACY_DEBUG` 需要外部真实 LIO/状态输入；`PRODUCTION` 会明确拒绝该 smoke launch。不要用静态 TF 绕过正式定位依赖。

### Point-LIO 健康失败

查看 `/sensors/front_mid360/cloud_reason`、`/state/lio_reason` 和 `/state/chassis_healthy`。常见原因是有效回波不足、点云源时间异常、IMU 单位/frame 不匹配或质量门断流。MID-360 原始 IMU 当前按 `g` 检查，不能把同一消息又乘一次 9.81；点云逐点 timestamp 是绝对纳秒，不能再与 header 时间相加。

### 物理串口没有输出

这是默认安全状态。确认 `my_serial_py` 的 PTY 测试先通过，再检查 `/nav/cmd_vel_safe`、`/nav/motion_enable`、`/state/chassis_healthy` 和 `/localization/healthy` 是否新鲜。当前框架没有把安全输出自动接到物理底盘，不能用 `/contact_angle` 代替速度或停车反馈。

### 远端构建失败

确认 shell 顺序为 `/opt/ros/jazzy` → `/home/asus/nav_deps/install_lio` → 本工程 `install`，并把并行度限制为 2。优先查看第一个编译错误；不要删除 `build/ install/ log/` 来掩盖依赖问题。修改后先运行受影响包的 smoke，再运行 `colcon test-result --verbose`。

## 阶段 B 硬件协议契约

当前 `my_serial_py` 的协议边界、缺失字段和兼容扩展方案见 [阶段 B 硬件协议契约](docs/stage_b_hardware_contract.md)。在收到真实轮速、实际速度、绝对云台角、停车确认、watchdog 和急停状态之前，硬件状态闭环保持 BLOCKED，不能把 Nav2 输出接到物理底盘。

## 开发约定

修改流程固定为：阅读架构契约 → 在 ASUS 构建 → 运行对应 smoke/MCAP → 更新文档和阶段矩阵 → 使用中文 commit → 推送 `audit-p0-tf-costmap` 等 feature branch。测试未完成的功能不能合并 `main`。提交身份统一为 `kswlt <kswlt@users.noreply.github.com>`。

