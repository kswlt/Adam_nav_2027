# Adam_nav_2027

面向 ASUS NUC 15 Pro、Ubuntu 24.04、ROS 2 Jazzy 的 RoboMaster 全向哨兵导航框架。
架构依据 `RM_Nav_V2` 技术方案 v2.5，通信复用此前实车运行的 `my_serial_py`。
构建与运行验证在 `asus@192.168.1.145` 完成，代码同步至
[kswlt/Adam_nav_2027](https://github.com/kswlt/Adam_nav_2027)。

## 当前进度

截至 2026-10-10，工程含 14 个 ROS 包，远端全量构建通过。
最近一次测试汇总为 36 项、0 失败、1 跳过；跳过项是可选的原版 libscrc 兼容核验，
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
Foxglove 查看地图、TF、原始点云及实时建图请看 [Foxglove 可视化说明](docs/foxglove_visualization.md)。

MPPI 基线已提供官方 `pointcloud_to_laserscan` 适配器，可将质量门后的 MID-360 点云转换为
Nav2 使用的 `/scan`；默认关闭，必须先完成实测 TF、传感器高度和 footprint 验收后再启用，
详见 [MPPI 点云适配说明](docs/mppi_pointcloud_scan_adapter.md)。

MID-360 已接通 ASUS，修复有线/无线重叠路由。原始点云质量门已接入，可拒绝有效回波不足、
几何退化、时间异常和断流，并联锁 LIO 健康；移除遮挡后每帧有效点约 1.41–1.48 万，
稳定段点云 10 Hz、IMU 200 Hz；真实 rosbag 静止回放约 20.7 s，最大位置变化 8.4 mm、
最大姿态变化约 0.09°，本次静止检查通过，动态和整车验收仍待完成。
接线配置、原始 rosbag 及遮挡失败对照见 [实机接入记录](docs/mid360_hardware_acceptance.md)。

## 构建（asus 主机）

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

## 已实现的启动与验收

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

## 包结构

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

## 验收记录与后续顺序

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
