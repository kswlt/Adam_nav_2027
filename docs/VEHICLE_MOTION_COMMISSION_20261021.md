# 1 米往返与物理速度调试记录

## 已完成的往返采集

用户说明：前进约 1 米，直接倒车返回，云台基本固定。没有精确尺量或外部真值，不按严格 1 米尺度验收。

ASUS 运行根目录为 `/home/asus/nav_2027_vehicle_config_164/runtime_20261021_sensor_reference`。

- MCAP：`bags/run_20261021_192709`，正常停止。
- 归档：`keyframes/session_1792582031013978757_3481820447`，60 帧，正常停止。
- LIO 999 条、参考里程计 1,005 条，覆盖约 103 秒；5,161 条状态健康心跳无 false。
- 雷达轴平面最大估计位移约 1.270m，末尾估计距起点约 0.079m；这不是有真值的 ATE。
- 原始 LIO 最大高度变化约 0.018m，末尾相对起点约 +0.001m；未重现前次抬升段的大幅高度变化，不代表已修复该失效机制。
- LIO 最大姿态变化约 14.4°、末尾约 -10.5°；原始 IMU 扣除首 20 秒均值零偏后近似 z 轴积分约 -10.1°。它支持存在传感器角运动，但不是姿态真值，不能宣称航向精度通过。

数值副本：`docs/evidence/roundtrip1m_20261021.json`。

## 实际建图后端

对完成的 60 帧运行现有 `offline_graph_optimizer --enable-loops`，实际 VGICP/KISS/GTSAM 执行通过，重建 199,877 个体素点；图目标函数 0.2949046 → 0.0279768。46 条 accepted loop 主要来自短距离和大量重复静止观测，不能解释为 46 次真实绕场闭环，也不能仅由目标函数下降证明地图准确。

墙钟 59.73s，汇总 CPU 655.72s，峰值 RSS 152,048KiB。原始档案保留。

输出 `optimized_roundtrip1m_20261021`，已生成 `draft.json`，状态仍为 `draft_mapping_odom`，不发布导航、不创建静态 map→odom，不把短路线当成完整场地图。

## 当前物理输出情况

用户明确授权自行操控，并确认在车旁可遥控接管、前后至少半米无人员障碍。新增独立有限点动工具 `tools/commission_sensor_xy.py`，复用原版 `my_serial_py.serial_bridge.SerialNode` 和协议编码；不会发布假的定位健康或 `/nav/motion_allowed`。

默认低速预算 0.05m/s × 最多 0.5s，普通调试最大 0.2m/s。后续按用户明确要求新增显式 `--high-speed-commission`：最大 1m/s、最长 1s、名义行程不超过 1m；生产适配器的 0.5m/s 限速不变。沿水平雷达朝向 +x，不保证等于车头 +x，扫描时方向会变化。检查真实 cloud/state/LIO、CRC-valid RX；实际估计位移超过本次名义行程加裕量时中止；每个正常或异常退出路径保持 2 秒新鲜零平移后关闭串口。不修改 yaw、底盘模式、区域业务字段，沿用 legacy cached 初始零。**这不等于固件全车停车反馈**，也不能停止电控自主 yaw 行为。

所有全局行为控制话题重映射到 `/commissioning/`，不消费原车 `/cmd_vel`；不连接 Nav2。物理端口必须显式选择 `--execute-physical`，测试端口只接受 `--test-only-pty` 且禁止 Domain 97。默认还要求雷达静止；用户明确说明云台自旋后，单独选择 `--rotating-gimbal`，仅取消不适用的雷达静止前提，其他条件不变。

PTY 实际回归通过：54 字节 Modbus CRC、25 个非零帧、非零窗口小于 0.6s、末尾至少 20 个连续零帧、yaw/默认字段未改变。测试输入是 Domain 215 的明确 TEST_ONLY fixture，未打开物理设备。

最初三次物理尝试都 `pulse_started=false`：第一次 LIO 已 input arrival gap；第二次因旋转雷达未满足静止条件；第三次 source age invalid，状态 false。串口 CRC-valid RX 持续正常，结束前只提交零 xy。后续结果见下方，不能将这三次拒绝解释为低速带不动车辆。工具里 completed 仅表示发送流程完成，不能独立作为真实底盘动作验收。

报告原件：`commission_sensor_xy_20261021_{first,retry,scanning}/report.json`。原始失败证据保留。

## 输入延迟排查

增加 guard 错误的 stream、source age/gap、arrival gap 数值，不改变阈值或锁止行为。连续性 11 项单测通过；ASUS rm_nav_bringup 构建通过。捕获 cloud age=0.324692s 超过 0.25s，未发现相应系统时钟跳变。

确认正在使用 rmw_fastrtps_cpp，Livox publisher 为普通 QoS 发布。本项目独立 driver 设置 `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`，不改原车 workspace 或驱动源码。[上游配置说明](https://github.com/ros2/rmw_fastrtps)。仅作为缓解实验，不宣称所有断流根因已解决。

随后 30 秒空闲 + 30 秒录包：初始化完成后 input_reason 保持 continuous original inputs；录包阶段 guarded cloud 最大 age 0.10535s，raw IMU 最大 age 0.00936s；最大 ROS/monotonic 增量差约 0.086ms。启动阶段 observer 有发现/丢样间隔，不把全阶段 max gap 说成源传感器断流。最初同步启动 bag 的 observer 会阻塞自己约 1s，已改为异步启动，旧报告保留且不能拿其瞬时 subscriber gap 当作驱动停流证据。

报告：`docs/evidence/latency_ab_async_20261021.json`。尚缺更长时间和多轮云台动态回归。

## 实际 1m/s 测试成功记录

用户随后明确确认周围各方向至少 2m 净空、可立即遥控接管，并要求延长起步时间。0.2m/s×0.3s 和 1m/s×0.2s 的流程完成后，用户认为时间不足；最终只执行一次 **1m/s×1s**，随后 2s 零平移，再关闭串口。

`strace -f -y -xx -s 96 -e trace=write` 核验 `/dev/ttyUSB0` 的实际内核写入：244 个成功的 54 字节报文，50 个非零报文，wire xy=(-1, -0)，CRC 全部正确，末尾 100 个连续零 xy。负号来自原版协议唯一取反，ROS 请求为 sensor +x=1m/s，yaw 双 float 仍为 0，stance=3/region=0/aligned=0，其他字段未改。

用户现场确认：**底盘明显移动，随后停下**。这验证了这一次实际平移响应及人工观察到的停车，尚无实际轮速或电控停车状态反馈，不声称精确行程、速度闭环、各方向坐标或自动导航通过。云台自旋期间 LIO 轨迹存在约 0.18m/s 的背景变化，不能把工具估计末位移约 0.35m 当成本次真实底盘行程。

报告：`docs/evidence/commission_1mps_1s_20261021.json`、`docs/evidence/commission_wire_verified_20261021.json`。原始 trace 和 MCAP 留在 ASUS runtime；所有物理工具已经退出、串口关闭，未留下持续非零输出或 Nav2 目标。
