# P0-P10 验收矩阵

| 阶段 | 当前证据 | 状态 |
| --- | --- | --- |
| P0 | 原版、技术方案、参考仓库审计；Jazzy 远端确认 | 已完成 |
| P1 | 14 个包可在 Jazzy 构建 | 已完成 |
| P2 | 原版冻结保留、分离串口桥与协议、CRC/断帧单测、ROS/PTY 集成 | 软件兼容性和超时停车通过；实车报文仍需下位机抓包确认 |
| P3 | TF 所有权、显式标定模板、绝对编码器时间戳 TF、ObservationBatch；MID-360 固定驱动及字段/时间/原始几何质量门 | 主 LIO 观测/源原点及退化输入拒绝验收通过；原始 IMU 为 g、frame 待适配；真实外参及多源待接入 |
| P4 | 固定上游 small_point_lio、完整 SE(3) ROS resolver、60 秒静止及明确约 1 m 移动的 MID-360/Foxglove 回放 | 静止链通过（最大变化 1.16 cm）；约 1 m 移动被捕获（估计最大位移 0.773 m），外部真值、实测标定及轮速待接入 |
| P5 | 真实GICP/KISS、双次候选、受限自动触发、停车/取消/TF/清图及新任务放行 | 主 LIO→滚动子地图→GICP 与恢复链已接入；真实回放与实车稳定性待验证 |
| P6 | 原始关键帧采集、真实 VGICP 相邻因子、GTSAM Pose3 优化与原始点云离线重建；有界回环候选和双向质量门 | 合成漂移归档及回环门单测通过；回环边接入、真实回环、官方对齐、清理及 MapBundle 发布待接入 |
| P7 | TimedTrajectory、路径预处理、Nav2 Smac2D + MPPI Omni launch；官方 MID-360 PointCloud2→LaserScan 可选适配 | 理想全向模型导航集成通过；适配器默认关闭，待实测 TF、footprint、Costmap 和真实环境验收 |
| P8 | SweptFootprint、CommandSynthesizer、MPPI → smoother → collision monitor | 横移导航及三种停车集成通过；TDT/Omni PID/串口适配待接入 |
| P9 | 本地/全局健康联锁、许可输出门、任务监管、恢复后新规划、外部抢占与故障保持 | 软件监管已接入；实车反馈、最终控制和运行验收待完成 |
| P10 | 全量构建、CTest、版本化配置、远端推送；Foxglove Bridge、实时话题清单和 PCD 可视化发布器 | 工程验证完成；框架完成后自行寻找公开 rosbag 系统测试、实车及长期验收待完成 |

## 不得误报为完成的项目

- 当前没有把 `my_serial_py` 改成新的串口协议。
- 当前没有伪造 small_gicp、KISS、GTSAM、TDT、MPPI 或 ros2_control 的算法输出。
- 当前单元测试证明的是数据契约和边界逻辑，不等于实车导航验收。
- 真实硬件输入、地图、标定、MCAP 和下位机报文仍是最终验收所需证据。
- 用户要求的公开 rosbag 系统测试为最终交付必做项，见 [执行计划](rosbag_system_test_plan.md)。
