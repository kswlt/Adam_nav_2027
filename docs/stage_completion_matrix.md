# P0-P10 验收矩阵

| 阶段 | 当前证据 | 状态 |
| --- | --- | --- |
| P0 | 原版、技术方案、参考仓库审计；Jazzy 远端确认 | 已完成 |
| P1 | 14 个包可在 Jazzy 构建 | 已完成 |
| P2 | `my_serial_py` 冻结复制、协议审计、硬件边界包 | 基线完成；实车报文仍需下位机抓包确认 |
| P3 | TF 所有权、CalibrationBundle、ObservationBatch | 接口完成；真实外参待测量 |
| P4 | SE(3) chassis resolver、LocalSubmap、robot state 边界 | 数学单测完成；真实 LIO/IMU/轮速接入待完成 |
| P5 | RegistrationResult、Validator、MapOdom、状态机、候选区域 | 状态与契约完成；small_gicp/KISS 后端待接入 |
| P6 | Keyframe、LoopCandidate、MapBundle 契约 | 编排接口完成；GTSAM/地图重建待接入 |
| P7 | TimedTrajectory、路径预处理、Stable/Enhanced profile | 契约完成；Nav2 真实 launch/参数待接入 |
| P8 | SweptFootprint、CommandSynthesizer | 单测完成；TDT/MPPI/Omni PID 运行链待接入 |
| P9 | NavSupervisor、硬件边界、传感器契约 | 状态与接口完成；真实控制器/传感器运行待接入 |
| P10 | 全量构建、CTest、版本化配置、远端推送 | 工程验证完成；实车回放和长期运行待完成 |

## 不得误报为完成的项目

- 当前没有把 `my_serial_py` 改成新的串口协议。
- 当前没有伪造 small_gicp、KISS、GTSAM、TDT、MPPI 或 ros2_control 的算法输出。
- 当前单元测试证明的是数据契约和边界逻辑，不等于实车导航验收。
- 真实硬件输入、地图、标定、MCAP 和下位机报文仍是最终验收所需证据。
