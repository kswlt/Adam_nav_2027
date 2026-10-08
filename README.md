# Adam_nav_2027

面向 ASUS NUC 15 Pro、Ubuntu 24.04、ROS 2 Jazzy 的 RoboMaster 全向哨兵导航框架。
架构依据 `RM_Nav_V2` 技术方案 v2.5，通信复用此前实车运行的 `my_serial_py`。
构建与运行验证在 `asus@192.168.1.145` 完成，代码同步至
[kswlt/Adam_nav_2027](https://github.com/kswlt/Adam_nav_2027)。

## 当前进度

截至 2026-10-09，工程含 14 个 ROS 包，远端全量构建通过。
最近一次测试汇总为 32 项、0 失败、1 跳过；跳过项是可选的原版 libscrc 兼容核验，
单独加载 libscrc 1.8.1 后全部 13 项串口协议测试通过。

| 功能 | 已验证内容 | 尚未完成 |
| --- | --- | --- |
| 原版通信 | 26-byte RX / 54-byte TX、原 CRC/符号/yaw/话题兼容、ROS 虚拟串口、平移超时停车 | 实车抓包、断线重连和下位机 watchdog 联调 |
| Nav2 基线 | Smac2D + MPPI Omni → smoother → collision monitor；横移导航、障碍/雷达断流/命令超时停车 | 实车定位输入、测量足迹、硬件速度与 yaw 适配 |
| 局部配准 | 真实 small_gicp、冻结地图裁剪、odom 子地图 ROS 匹配、质量/时间/版本拒绝测试 | 目标 KD-tree 缓存、真实点云回放、KISS 恢复 |
| TF 与定位 | SE(3) resolver、独占 map→odom 节点、健康超时、健康/许可控制输出门 | small_point_lio、动态云台编码器、EKF、完整任务权限节点 |
| 优化建图 | 关键帧、回环候选、MapBundle 基础数据结构 | KISS 回环复核、GTSAM 优化、原始关键帧地图重建 |
| 最终规划控制 | 时序轨迹、路径与足迹验证基础接口 | TDT 后端、Omni PID、YawManager、ros2_control 适配 |

**P0–P10 尚未全部完成。** 数据结构和单元测试不等于完整导航功能。
详细进度见 [阶段验收矩阵](docs/stage_completion_matrix.md)。

## 构建（asus 主机）

```bash
cd /home/asus/nav_2027/rm_nav_v2
source /opt/ros/jazzy/setup.bash
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_small_gicp.sh
CMAKE_PREFIX_PATH=/home/asus/nav_deps/install:${CMAKE_PREFIX_PATH:-} colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

远端已安装 Nav2 bringup/MPPI/collision monitor/velocity smoother、robot_localization、
ros2_control 和 controllers。small_gicp 在用户目录构建，精确提交见
[dependencies.repos](dependencies.repos)。[依赖清单](dependencies.lock.yaml) 目前仅部分锁定。

## 已实现的启动与验收

```bash
# 独立 Domain 中运行理想全向模型，启动真实 Nav2 节点。
ROS_DOMAIN_ID=87 python3 tools/smoke_mppi_baseline.py

# 伪终端模拟 STM32，启动真实串口 ROS 节点，不打开物理串口。
ROS_DOMAIN_ID=88 python3 tools/smoke_serial_pty.py

# 真实 GICP 算法，验证已知变换和异常数据拒绝。
./build/rm_nav_registration/small_gicp_backend_test

# 真实配准和 TF 节点，模拟冻结地图及 odom 子地图；时间/版本/大修正拒绝。
ROS_DOMAIN_ID=89 python3 tools/smoke_frozen_map_localization.py

# 失健康、失许可、心跳中断、无缓存重放与非法命令拒绝。
ROS_DOMAIN_ID=90 python3 tools/smoke_motion_gate.py
```

Nav2 外部输入：`/map`、`/odom`、`/scan` 和 `map → odom → base_link` TF。
启动：`ros2 launch rm_nav_bringup mppi_baseline.launch.py`。
输出链 `/nav/cmd_vel_raw → /nav/cmd_vel_smoothed → /nav/cmd_vel_checked → /nav/cmd_vel_safe`
均为 `TwistStamped`、底盘坐标速度。当前输出隔离，尚未自动接到串口。
footprint、停止区和速度约束目前为调试初值，需要实际尺寸与制动数据。
最终输出门要求新鲜的 `/localization/healthy` 和 `/nav/motion_enable` true 心跳，
以及新鲜有效的底盘坐标速度；默认零输出。运动许可的任务/监管发布节点仍待实现。
见 [运动许可验收](docs/motion_gate_acceptance.md)。

冻结地图定位启动：
`ros2 launch rm_nav_bringup frozen_map_localization.launch.py map_version:=<实际地图版本>`。
输入 `/localization/frozen_map`（map 坐标、transient local）和
`/localization/odom_submap`（odom 坐标、源观测时间戳、已去畸变）。
输出 `/localization/estimate`、`map → odom` TF、`/localization/healthy` 和
`/localization/correction_pending`。大修正当前只保留候选；双重确认与停车重规划还待实现。
详细约定见 [冻结地图定位验收](docs/frozen_map_localization_acceptance.md)。

串口启动：
`ros2 launch my_serial_py serial.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200 cmd_vel_timeout:=0.3`。
运行入口 `serial_bridge.py`；原版 `serialpy_node.py` 保留用于对照。
`/cmd_vel` 仅提供 xy 平移；`/cmd_yaw_angle` 提供 yaw 目标角（度），
报文不发送 `angular.z`。因此现有平移 watchdog 不能替代全底盘停车联调。

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
- [冻结地图 ROS 定位链](docs/frozen_map_localization_acceptance.md)
- [定位健康与运动许可](docs/motion_gate_acceptance.md)
- [串口字段说明](docs/my_serial_py_interface.md)
- [架构与 TF 契约](docs/architecture_contract.md)

下一步接入 KISS 丢失恢复、任务权限节点，以及大修正的独立确认和停车重规划；
之后完成真实传感器/LIO 链路、优化建图、TDT 和最终控制链。
每个独立步骤先在 asus 验证，再以中文 commit 推送。提交身份统一为 `kswlt`。
