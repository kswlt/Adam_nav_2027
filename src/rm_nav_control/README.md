# rm_nav_control

Controller baseline, Omni PID and yaw authority.

真实 ROS 入口 `motion_gate`：checked 命令经定位健康、运动许可与时间/数值检查后
输出 safe 命令。默认停车，失效清空缓存，恢复必须接收新命令。
`omni_pid_adapter` 已提供 Omni PID、YawManager 和 `my_serial_py` 字段适配：输入
`/nav/cmd_vel_safe`，读取真实 `/hardware/measured_twist`、`/hardware/drive_ready` 和 `/odom`，
输出 `/cmd_vel`（仅 XY）与 `/cmd_yaw_angle`（degree）。`enable_physical_output` 默认 false，
关闭物理输出时不发布命令。开启后，输入不完整或过期时输出零平移并清空旧命令，
只有新命令才能恢复。它不把 EKF 估计速度冒充实测速度。
见 [输出门验收](../../docs/motion_gate_acceptance.md)。

在 MPPI launch 中启动软件适配器：

```bash
ros2 launch rm_nav_bringup mppi_baseline.launch.py \
  enable_omni_adapter:=true \
  enable_physical_output:=false
```

只有真实反馈、STM32 watchdog、独立急停和人工批准都完成后，才允许单独评审
`enable_physical_output:=true`；默认配置不会发布物理底盘命令。

`CommandSynthesizer` clamps world-frame velocity commands and emits zero during a safety gate. It does not own serial transport or strategy policy.

## 输入与失效处理

- 默认 `require_measured_feedback=true`，使用真实 `/hardware/measured_twist` 作 PID 修正。
- 用户暂缓轮速反馈时可显式设置 `require_measured_feedback=false`：纯速度前馈，完全绕过 PID 修正；仍需真实 `/odom` 与 `/hardware/drive_ready` 心跳。
- TwistStamped 必须在 `base_frame`、数值有限且为平面运动，源时间不得为零、负值、未来或过期。
- `/odom` 必须为 `odom_frame → base_frame`，四元数归一化，时间戳及接收时间均新鲜。
- 就绪 Bool 必须持续刷新。默认命令/里程计/就绪超时 0.25 秒，轮速超时 0.2 秒。
- 前馈与 PID 叠加后，最终 XY 向量模长限制为 `max_planar_speed`（默认 0.5 m/s），yaw 速率限制为 `yaw.max_rate`（默认 1 rad/s）。这是软件限幅，尚未实车调参。
- safe 全零命令直接输出零平移并以当前实测 yaw 重置角目标，不叠加 PID 制动修正。
- 失效且里程计仍新鲜时输出当前角度用于保持；里程计失效时停止更新 yaw 目标。**原协议及 serial_bridge 会保留旧 yaw 角，不能据此宣称旋转已停车。必须核验下位机停止/失能语义。**

MPPI launch 现在读取 `omni_params_file`（默认 `omni_pid_adapter.yaml`），支持显式
`require_measured_feedback` 参数。输出默认关闭，PID 参数默认零，保持通信协议不变。

`omni_pid_adapter_regression` 在独立 ROS Domain 199、localhost discovery 下运行，
所有话题重映射到 `/test_only/omni/*`，不启动串口。覆盖无反馈模式、最终限幅、
命令/状态/就绪/轮速超时、非法时间和四元数、恢复不重放及零速请求。
