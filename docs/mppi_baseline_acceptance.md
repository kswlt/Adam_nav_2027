# MPPI Omni 软件基线验收

2026-10-09，在 asus（Ubuntu 24.04 / ROS 2 Jazzy）完成真实 Nav2 节点集成测试。
安装版本：Nav2 1.3.13。此记录针对理想全向运动模型，不代表实车验收。

## 启动与接口

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup mppi_baseline.launch.py
```

外部必须提供 `/map`（transient local）、`map → odom → base_link`、`/odom`、`/scan`。
本 launch 不发布定位 TF，保证后续 MapOdomManager 的独占所有权。
SmacPlanner2D 负责全局路径，MPPI 使用 Omni 运动模型，无 PreferForwardCritic。
速度链为 `/nav/cmd_vel_raw → /nav/cmd_vel_smoothed → /nav/cmd_vel_safe`，
均为 `geometry_msgs/TwistStamped`，速度属于底盘 `base_link` 坐标。
最终输出没有连接 `/cmd_vel` 或串口；硬件接入需要实现坐标和 yaw 语义适配。

当前 footprint、停止区域和速度上限是调试初值，必须用实测底盘尺寸及制动距离替换。
当前观测采用 LaserScan；多雷达保留观测原点的 PointCloud2/VoxelLayer 接入尚未完成。

## 复现

在空闲的 ROS Domain 中运行：

```bash
ROS_DOMAIN_ID=87 python3 tools/smoke_mppi_baseline.py
```

脚本启动真实六个生命周期节点，并在退出时关闭该 launch 的进程组。
模拟节点仅在该 Domain 发布理想地图、TF、里程计和 LaserScan，不启动硬件或串口。
报告输出至 `log/mppi_baseline_smoke.json`，完整 ROS 日志在同名 `.log`。

## 本次结果

- controller、planner、behavior、BT、smoother、collision monitor 均激活。
- `NavigateToPose` 从 `(0,0,0)` 到 `(0,1,0)`，返回 SUCCEEDED（status 4）。
- 到达 `(0.02064,0.86462,-0.03903)`，满足 0.15 m / 0.2 rad 配置容差。
- 最大横向速度 `0.27443 m/s`，证明非零 y 速度通过完整输出链。
- 0.25 m 障碍点进入停止多边形后输出归零，通过。
- 继续注入运动命令并停止雷达发布，超过 0.3 s 超时后输出归零，通过。
- 恢复雷达后停止速度命令，smoother 命令超时停车，通过。

这是 P7/P8 的软件基线证据；TDT、最终 Omni PID、真实定位和 ros2_control 仍待接入。
