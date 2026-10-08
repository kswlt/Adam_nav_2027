# RM Nav V2 Architecture Contract

## Frozen interfaces

第一阶段冻结以下 `my_serial_py` 接口：

- `/cmd_vel`：底盘平移速度输入。
- `/cmd_stance`：底盘运行姿态输入。
- `/cmd_yaw_angle`：下位机 yaw 目标输入。
- `/region`：区域编码输入。
- `/big_yaw_aligned`：大云台对齐标志输入。
- `/referee/robot_status`、`/referee/game_status`、`/referee/all_robot_hp`、`/referee/rfid_status`：裁判状态输出。
- `/contact_angle`、`/is_fire`：下位机反馈输出。

当前协议和话题来自原版 `串口通信说明.md` 及 `my_serial_py/serialpy_node.py`。任何协议变化必须同时更新上位机、下位机说明、回放测试和接口记录。

## TF authority

```text
map -> odom                 MapOdomManager
odom -> base_footprint      ChassisState / robot_localization
base_footprint -> chassis   static calibration
chassis -> big_gimbal_yaw   gimbal state
big_gimbal_yaw -> lidar     CalibrationBundle
```

重定位只能修正 `map -> odom`，不得重置 Point-LIO 或跳变 `odom -> base_footprint`。

## Stable Baseline

先完成：small_point_lio、dynamic-gimbal resolver、robot_localization、Nav2 Costmap、Nav2 Planner、MPPI Omni、Velocity Smoother、Collision Monitor 和 `my_serial_py`。

## 技术方案要求的完整能力

基线之后继续实现 small_gicp 局部配准、KISS 丢失恢复、验收器、MapOdomManager、
OPTIMIZED_BUILD 的 GTSAM 位姿优化与原始关键帧地图重建，以及 TDT 轨迹后端和最终 Omni PID。
这些属于技术方案的目标能力，不能因为 MPPI 基线通过而省略。
ROG-Map/ESDF、动态障碍预测和 ScanContext 等增强项按技术方案中的启用条件接入。

## 不重复实现

不自研通用 TF、EKF、ICP/GICP、图优化器、Nav2 Costmap、Nav2 Controller、QP 求解器、速度平滑器、碰撞监视器和诊断协议。

## 必须自研

动态云台底盘 resolver、Sensor Hub glue、关键帧编排、MapBundle 事务层、场地坐标对齐、窄道估计、扫掠足迹验证、Yaw Manager、Omni PID、NavSupervisor 和 STM32 硬件适配。
