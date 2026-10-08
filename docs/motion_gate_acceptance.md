# 定位健康与运动许可输出门验收

2026-10-09，在 asus / Jazzy 验证真实 C++ `motion_gate` 节点。
MPPI 启动文件现在包含该节点，命令链为：

```text
MPPI/behavior → /nav/cmd_vel_raw → velocity_smoother
 → /nav/cmd_vel_smoothed → collision_monitor
 → /nav/cmd_vel_checked → motion_gate → /nav/cmd_vel_safe
```

全部速度消息为 `TwistStamped`，底盘坐标 `base_link`。
最终输出仍未接入串口或真实 ros2_control 硬件。

## 放行条件

- `/localization/healthy` 为 true，接收心跳不超过 0.25 s。
- `/nav/motion_enable` 为 true，接收心跳不超过 0.25 s。
- checked 命令接收时间和源时间戳不超过 0.25 s，未来时间不超过 0.1 s。
- frame 为 base_link，六个速度字段有限，z 平移和 roll/pitch 角速度为零。

否则每 20 ms 发布零速度。显式失健康、禁用或超时清空缓存；恢复后必须收到新命令。
平移按二维向量范数限至 0.5 m/s，yaw 角速度限至 ±1 rad/s，均为调试初值。
心跳接收使用 steady clock，不依赖 ROS clock；命令观测时间仍用 ROS clock 检查。
健康/许可订阅采用 volatile QoS，避免读取失效发布者留下的历史 true。
输出 `/nav/motion_allowed` 和 `/nav/motion_gate_reason` 用于诊断。

实际定位链的健康发布者是 `map_odom_manager`。
运动许可应由任务/监管的唯一权限节点周期发布，当前尚未实现完整 NavSupervisor/BT 集成。
没有该许可源时保持零输出，不在启动文件中自动造一个恒 true 许可。
理想模型测试会显式模拟健康与许可心跳。

## 验收

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=90 python3 tools/smoke_motion_gate.py
ROS_DOMAIN_ID=87 python3 tools/smoke_mppi_baseline.py
```

独立输出门测试通过：启动停车、显式失健康、健康心跳中断、显式禁用、许可心跳中断、
恢复不重放缓存、命令超时、NaN、错误 frame、过期/未来时间、非平面速度拒绝，以及向量/yaw 限幅。

完整 Nav2 理想全向模型测试通过：
六个生命周期节点激活；横移 NavigateToPose 成功，最大横向速度约 0.27219 m/s。
原有障碍点、雷达断流、速度超时停车保持通过；新增失健康、健康心跳中断、
撤销运动许可时，即使继续注入速度指令，最终输出仍归零。

本次还将生命周期验收客户端改为复用，避免频繁创建/销毁服务客户端造成通信扰动。
日志保存在 `log/motion_gate_smoke.log` 与 `log/mppi_baseline_smoke*`。
大修正确认、停车状态确认、Nav2 目标取消/轨迹作废和重规划事务仍待后续实现。
