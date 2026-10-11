# rm_nav_control

Controller baseline, Omni PID and yaw authority.

真实 ROS 入口 `motion_gate`：checked 命令经定位健康、运动许可与时间/数值检查后
输出 safe 命令。默认停车，失效清空缓存，恢复必须接收新命令。
`omni_pid_adapter` 已提供 Omni PID、YawManager 和 `my_serial_py` 字段适配：输入
`/nav/cmd_vel_safe`，读取真实 `/hardware/measured_twist`、`/hardware/drive_ready` 和 `/odom`，
输出 `/cmd_vel`（仅 XY）与 `/cmd_yaw_angle`（degree）。`enable_physical_output` 默认 false，
缺少真实反馈或 drive_ready 时不发布任何物理命令。它不把 EKF 估计速度冒充实测速度。
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
