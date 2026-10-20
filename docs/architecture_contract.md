# RM Nav V2 Architecture Contract

## 雷达参考导航 profile（2026-10-21）

新增独立传感器参考模式：LIO只观测雷达6DoF；nav_reference为雷达轴平面位置、odom对齐朝向；sensor_yaw_reference为水平雷达朝向。没有角度反馈时不发布真实 chassis/base_link 姿态。Nav2/TDT只控制平移，电控承担自旋和大yaw；影子输出不接串口。既有完整底盘 profile 的唯一 TF authority 和编码器要求保持不变。完整坐标、速度作用点与兼容合同见 [SENSOR_REFERENCE_MODE](SENSOR_REFERENCE_MODE.md)。

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
odom -> base_footprint      base_footprint_projection
base_footprint -> chassis   base_footprint_projection (dynamic 6DoF residual)
chassis -> big_gimbal_yaw   gimbal state
big_gimbal_yaw -> lidar     CalibrationBundle
```

robot_localization 保留完整 `odom -> chassis` 状态并输出 `/state/chassis`，不直接发布 TF；
base_footprint_projection 将同一时刻的 chassis 状态投影为平面 `odom -> base_footprint`，
并发布 `base_footprint -> chassis` 的动态 z/roll/pitch 残差。重定位只能修正 `map -> odom`，
不得重置 Point-LIO 或跳变 `odom -> base_footprint`。

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
