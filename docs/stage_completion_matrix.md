# P0-P10 验收矩阵

2026-10-21 ASUS新参考模式已部署：新release隔离，ASUS56 tests / 0 errors / 0 failures / 1 skipped；真实状态及WebSocket预览通过，关键帧与在线地图运行。只读电控接收正常；物理发送、正式地图定位和自动行驶尚未验收。见 [ASUS现场记录](ASUS_SENSOR_REFERENCE_20261021.md)。

2026-10-21：新增雷达轴参考模式，保留完整 chassis 模式。支持独立状态链、nav_reference 关键帧/在线优化、TDT只跟踪平移和雷达水平坐标速度影子转换。当前只在本机测试，ASUS部署及物理控制未执行。详见 [模式与验收记录](SENSOR_REFERENCE_MODE.md)。

2026-10-20 在线建图更新：后台读取已提交关键帧，双向 KISS/GICP + 既有 GTSAM 后验验收，
回环接纳后修正历史轨迹并替换全历史体素地图；独立 mapping_odom 可视化，odom/生产 map TF 不变。
ASUS 全 14 包构建、50 tests / 0 errors / 0 failures / 1 skipped；在线 ROS fixture 与原离线闭环回归通过。
当前实测 29 帧/6 条接纳边/无积压，真实场地回环精度和长期资源仍 NOT VERIFIED。
操作与证据见 [在线建图](ONLINE_MAPPING.md)，旧阶段表的离线说明保留为历史记录。

2026-10-20 更新：本机 WSL 与 ASUS 完整 14 包构建、49 tests / 0 failures / 1 skipped。
TDT 上游/Nav2 控制器、真实 Costmap 快照、Omni PID/YawManager、HardwareInterface 实际 ROS 读写、
租约与标准控制器、正式地图软件发布/坐标拟合均已实现。参见
[导航集成验收](NAVIGATION_INTEGRATION_ACCEPTANCE.md) 与 [ASUS 现场启动](ASUS_LIVE_20261020.md)。
下面旧阶段记录保留为历史证据；实车验收不能由软件通过替代，Git 已恢复 kswlt 身份推送。

第一阶段基础自主导航审计矩阵见 [phase1_function_audit.md](phase1_function_audit.md)。
当前软件闭环可以继续回归，但真实标定、硬件速度/停车反馈、正式地图和独立急停仍是 BLOCKED，
不能把合成测试或静止回放升级为实车运动验收。

| 阶段 | 当前证据 | 状态 |
| --- | --- | --- |
| P0 | 原版、技术方案、参考仓库审计；Jazzy 远端确认 | 已完成 |
| P1 | 14 个包可在 Jazzy 构建 | 已完成 |
| P2 | 原版冻结保留、分离串口桥与协议、CRC/断帧单测、ROS/PTY 集成 | 软件兼容性和超时停车通过；实车报文仍需下位机抓包确认 |
| P3 | TF 所有权、显式标定模板、绝对编码器时间戳 TF、ObservationBatch；MID-360 固定驱动及字段/时间/原始几何质量门 | 主 LIO 观测/源原点及退化输入拒绝验收通过；原始 IMU 为 g、frame 待适配；真实外参及多源待接入 |
| P4 | 固定上游 small_point_lio、完整 SE(3) ROS resolver、60 秒静止及明确约 1 m 移动的 MID-360/Foxglove 回放 | 静止链通过（最大变化 1.16 cm）；约 1 m 移动被捕获（估计最大位移 0.773 m），外部真值、实测标定及轮速待接入 |
| P5 | 真实GICP/KISS、双次候选、受限自动触发、停车/取消/TF/清图及新任务放行 | 主 LIO→滚动子地图→GICP 与恢复链已接入；真实回放与实车稳定性待验证 |
| P6 | 原始关键帧、VGICP/KISS/GTSAM、后验残差/形变/移除回环对照、100/300/500 帧归档优化、官方坐标拟合与 Bundle 发布软件 | 软件通过；真实场地回环、官方测量资料、地图清理与正式场地 Bundle 未验收 |
| P7 | TimedTrajectory、路径预处理、Nav2 Smac2D + MPPI Omni launch；PointCloud2→LaserScan；legacy 动态 TF；测试地图下 map_server、global/local Costmap 激活；复用原版 0.2 m 方形 footprint 基线 | 仍需正式场地地图、实测标定、footprint、障碍物位置和制动距离验收 |
| P8 | 真 TDT/Nav2、polygon swept 碰撞、Omni PID/YawManager、互斥 direct/ros2_control、原协议 PTY | 软件和 ASUS 理想底盘通过；实车 yaw、速度反馈与制动未验收 |
| P9 | 本地/全局健康联锁、许可输出门、任务监管、恢复后新规划、外部抢占与故障保持 | 软件监管已接入；实车反馈、最终控制和运行验收待完成 |
| P10 | 本机及 ASUS 14 包/49 tests、中文提交、SDK WebSocket 实际点云、公开 Koide 277 秒 MID360 回放 | 软件/传感器链证据已补齐；无真值精度、全车及长期运行验收仍待完成 |

## 不得误报为完成的项目

- 当前没有把 `my_serial_py` 改成新的串口协议。
- 当前没有伪造 small_gicp、KISS、GTSAM、TDT、MPPI 或 ros2_control 的算法输出。
- 当前单元测试证明的是数据契约和边界逻辑，不等于实车导航验收。
- 真实硬件输入、地图、标定、MCAP 和下位机报文仍是最终验收所需证据。
- 用户要求的公开 rosbag 系统测试为最终交付必做项，见 [执行计划](rosbag_system_test_plan.md)。
