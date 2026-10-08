# P0-P10 验收矩阵

| 阶段 | 当前证据 | 状态 |
| --- | --- | --- |
| P0 | 原版、技术方案、参考仓库审计；Jazzy 远端确认 | 已完成 |
| P1 | 14 个包可在 Jazzy 构建 | 已完成 |
| P2 | 原版冻结保留、分离串口桥与协议、CRC/断帧单测、ROS/PTY 集成 | 软件兼容性和超时停车通过；实车报文仍需下位机抓包确认 |
| P3 | TF 所有权、CalibrationBundle、ObservationBatch | 接口完成；真实外参待测量 |
| P4 | SE(3) chassis resolver、LocalSubmap、robot state 边界 | 数学单测完成；真实 LIO/IMU/轮速接入待完成 |
| P5 | 真实GICP/KISS、双次候选、受限自动触发、停车/取消/TF/清图及新任务放行 | 软件恢复链已接入；真实回放与实车稳定性待验证 |
| P6 | Keyframe、LoopCandidate、MapBundle 契约 | 编排接口完成；GTSAM/地图重建待接入 |
| P7 | TimedTrajectory、路径预处理、Nav2 Smac2D + MPPI Omni launch | 理想全向模型导航集成通过；环境查询和过滤层待接入 |
| P8 | SweptFootprint、CommandSynthesizer、MPPI → smoother → collision monitor | 横移导航及三种停车集成通过；TDT/Omni PID/串口适配待接入 |
| P9 | 健康/许可输出门、任务监管、恢复后新规划、外部抢占与故障保持 | 软件监管已接入；实车反馈、最终控制和运行验收待完成 |
| P10 | 全量构建、CTest、版本化配置、远端推送 | 工程验证完成；实车回放和长期运行待完成 |

## 不得误报为完成的项目

- 当前没有把 `my_serial_py` 改成新的串口协议。
- 当前没有伪造 small_gicp、KISS、GTSAM、TDT、MPPI 或 ros2_control 的算法输出。
- 当前单元测试证明的是数据契约和边界逻辑，不等于实车导航验收。
- 真实硬件输入、地图、标定、MCAP 和下位机报文仍是最终验收所需证据。
