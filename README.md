# Adam_nav_2027

面向 ASUS NUC 15 Pro + Ubuntu 24.04 + ROS 2 Jazzy 的哨兵导航框架。

本工程以 `RM_Nav_V2` 技术方案 v2.5 为架构基线，以原版 `my_serial_py` 为当前上位机与 STM32 下位机通信基线。第一阶段保持通信包名、消息类型、话题和报文格式不变，导航模块通过标准 ROS 接口接入。

## 当前状态

- P0/P1：工程骨架、通信基线和接口文档已建立。
- `src/my_serial_py`：从原版工作区复制，暂不修改协议。
- `src/pb_rm_interfaces`：通信节点所需消息包。
- Stable Baseline：尚未开始构建，需在远端 Jazzy 主机验证。

## 系统边界

```text
Point-LIO / Sensor Hub
        -> chassis resolver / robot_localization
        -> map-odom localization
        -> Nav2 planner + controller
        -> /cmd_vel, /cmd_stance, /cmd_yaw_angle, /region
        -> my_serial_py
        -> STM32
```

`my_serial_py` 是现阶段唯一的下位机通信主线。`standard_robot_pp_ros2` 不属于本工程第一阶段依赖。

## 目录

| 目录 | 职责 |
| --- | --- |
| `src/rm_nav_localization` | LIO 适配、动态云台 resolver、重定位 |
| `src/rm_nav_registration` | small_gicp/KISS 统一适配 |
| `src/rm_nav_mapping` | 关键帧、子地图、回环、GTSAM、MapBundle |
| `src/rm_nav_perception` | Sensor Hub、地形、窄道和环境适配 |
| `src/rm_nav_planning` | Nav2 路径预处理、TDT 轨迹后端、足迹验证 |
| `src/rm_nav_control` | MPPI 基线、Omni PID、Yaw Manager |
| `src/my_serial_py` | 原版串口节点，保持现有协议 |
| `docs` | 接口、架构和阶段验收记录 |

