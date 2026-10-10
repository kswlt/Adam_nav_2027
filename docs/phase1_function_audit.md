# 第一阶段基础自主导航功能审计矩阵

审计基线：远端 `audit-p0-tf-costmap` 当前提交 `1d8a89a`；审计前已执行 `git fetch origin`。
ASUS 工作区 `/home/asus/nav_2027/rm_nav_v2` 另有未提交的
`src/rm_nav_mapping/src/offline_graph_optimizer.cpp` overlap 优化修改，本审计不覆盖、不回退该修改。

状态含义：

- **软件通过**：源码、单测或 smoke 能证明接口行为。
- **已接线**：launch 和 ROS 话题已经串起来，但不代表真实硬件验收。
- **BLOCKED**：需要真实测量、下位机字段、正式地图或人工硬件验收，不能用替身补齐。
- **NOT VERIFIED**：缺少当前数据证据，不能从 README 推断通过。

| 功能 | 代码位置与当前实现 | 软件测试 | 真实硬件测试 | 当前判定与下一步 |
| --- | --- | --- | --- | --- |
| TF 拓扑和唯一发布者 | `src/rm_nav_bringup/launch/local_state.launch.py`；`scripts/gimbal_tf.py`；`map_odom_manager_node.cpp` | `smoke_local_state.py`、TF smoke 已通过；LIO 不发布 TF，EKF 发布 odom 链，MapOdomManager 独占 map→odom | 未完成云台旋转、底盘运动、断流组合测试 | **软件通过 / 硬件 BLOCKED**；需要实测编码器和完整 TF authority 录包 |
| CalibrationBundle 门 | `rm_nav_frames/config/local_state_calibration.template.yaml`、`validate_calibration_bundle.py`、`local_state.launch.py` | 未 verified Bundle 会拒绝；legacy 需要显式参数 | 当前仅使用 legacy 原版值联调，没有实测 Bundle | **软件通过 / 生产 BLOCKED**；测量四段外参、零位、方向和时钟误差 |
| 源时间 SE(3) resolver | `sensor_pose_resolver_node.cpp` | 检查 frame、单调时间、年龄、协方差正定、精确时间 TF 和质量门健康 | MID-360 静止回放通过；动态云台时间覆盖未验证 | **软件通过 / 动态 NOT VERIFIED**；采集云台往复和底盘运动数据 |
| 6DoF 状态与 Nav2 参考点 | `chassis_state_bridge_node.cpp`、robot_localization 参数生成 | `two_d_mode=false`，保留 z/roll/pitch；参考点变换和协方差检查有单测 | 没有轮速、底盘 IMU 或实际停车反馈 | **软件通过 / 硬件 BLOCKED**；接入真实状态后再确认参考点和速度方向 |
| MID-360 质量门与 LIO | `rm_nav_sensors`、`small_point_lio` 上游 overlay、`local_state.launch.py` | 原始字段、源时间、有效点、几何退化、断流联锁已有回归 | 有静止/遮挡对照；直线和旋转真值不足 | **静止通过 / 动态 NOT VERIFIED**；采集明确距离、角度和往返序列 |
| my_serial_py 通信 | `src/my_serial_py/my_serial_py/protocol.py`、`serial_bridge.py` | RX 26 bytes、TX 54 bytes、CRC、PTY、平移超时停车通过 | 未完成 STM32 抓包、重启、断线重连和 watchdog 确认 | **软件通过 / HIL BLOCKED**；需真实报文和下位机停车反馈 |
| 底盘/云台硬件状态 | `src/rm_nav_hardware/README.md` | 目前只有边界说明，没有虚拟状态冒充真实反馈 | 未提供轮速、实际速度、绝对云台角、硬件急停确认 | **BLOCKED**；需要协议字段或固件适配方案，不能生成 `/hardware/measured_twist` |
| 正式地图与 MapBundle | `rm_nav_mapping`、`tools/create_map_bundle.py`、`frozen_map_localization.launch.py` | 已增加二进制 XYZ PCD 头/点数/有限值、输入哈希记录、连续 pose ID、draft 发布门和原子清单 smoke | 没有官方场地坐标、正式 PCD/PGM/terrain 和真值 | **软件部分通过 / 生产 BLOCKED**；完成对齐、清理、原子发布和人工验收 |
| GICP/KISS 定位与恢复 | `frozen_map_matcher_node.cpp`、`map_odom_manager_node.cpp`、registration backend | 双候选、恢复事务、旧任务取消和失败保持 smoke 通过；回环 overlap 已复用 small_gicp KdTree，离线建图回归通过 | 重复结构、长时间漂移和真实停车反馈未验证 | **软件通过 / 实车 BLOCKED**；使用真实场地和实测停止反馈回放 |
| Costmap 与 Nav2 MPPI | `costmap_mid360_smoke.launch.py`、`mppi_baseline.launch.py`、Nav2 参数 | TEST_ONLY/LEGACY_DEBUG/PRODUCTION 隔离；软件障碍和断流停车通过 | footprint、障碍空间位置、正式地图和制动距离未测 | **软件基线通过 / 实车 BLOCKED**；替换实测 footprint 和安全限值 |
| 任务监管与安全门 | `task_supervisor.py`、motion gate、recovery transaction | 健康断流、恢复、旧任务取消、重新规划 smoke 通过 | 无独立急停、下位机 watchdog 和真实速度确认 | **软件通过 / 上车 BLOCKED**；完成 HIL 后才可申请低速运动 |
| Foxglove 诊断 | `foxglove_visualization.launch.py`、live preview/trace publishers | 话题、TF、PCD、实时预览链已验证 | 无法由可视化证明定位精度或制动性能 | **可用**；高带宽点云只开一路以控制延迟 |

## A 阶段审计结论

当前可以继续完成的软件工作包括：TF authority 的自动检查、时间/断流故障回放、串口 PTY/HIL 框架、MapBundle 输入校验、真实 rosbag 回放和 Costmap 参数化。以下条件未满足前，阶段结果只能标记 `BLOCKED`：

1. 没有实测 CalibrationBundle 和绝对云台编码器数据，不能宣布生产定位链通过。
2. 没有轮速/实际速度/停车确认和独立 watchdog，不能把 Nav2 输出接入物理底盘。
3. 没有官方场地坐标与正式地图，不能把 `mapping_odom` PCD 作为冻结 `map`。
4. 没有独立急停和人工批准，不能执行电机使能或自主运动。

下一步按任务要求进入 B 阶段的软件审查：先把现有 `my_serial_py` 协议字段、命令单位、超时和断线状态整理成可执行的 HIL/PTY 验收，不添加伪造反馈；然后再根据真实 STM32 字段决定薄适配层能否实现。
