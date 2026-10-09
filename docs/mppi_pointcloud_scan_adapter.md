# MPPI 基线的 MID-360 障碍物适配

`mppi_baseline.launch.py` 提供了一个可选的官方
`pointcloud_to_laserscan` 适配器，将 MID-360 的受质量门保护点云转换为
Nav2 Costmap 使用的 `/scan`。

默认参数 `enable_mid360_scan:=false`，因此不会在未完成实测标定时误把点云
接入规划器。完成 `base_link`（或指定目标 frame）与雷达 frame 的实测 TF、
高度范围和机器人 footprint 验收后，再使用：

```bash
ros2 launch rm_nav_bringup mppi_baseline.launch.py \
  enable_mid360_scan:=true \
  pointcloud_topic:=/sensors/front_mid360/guarded_points \
  scan_target_frame:=base_link
```

适配器只负责消息类型转换；它不替代真实 CalibrationBundle、点云质量门、
定位健康联锁或最终 Omni 控制器。Foxglove 中应同时确认 `/scan` 的时间戳、
`frame_id`、TF 连续性和 Costmap 障碍物位置，再进行 MPPI 障碍物停止测试。
