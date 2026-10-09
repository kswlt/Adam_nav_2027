# MID-360 Costmap 联调

测试地图与完整联调启动：

```bash
ros2 launch rm_nav_bringup costmap_mid360_smoke.launch.py \
  map_yaml:=/tmp/smoke_map.yaml
```

该启动器自动管理 `map_server` 生命周期，并启动原版静态外参联调模式、
Point-LIO/EKF 状态链和 MPPI 的点云转 LaserScan 适配。它只验证消息、TF、
地图和 Costmap 链路，不连接 `my_serial_py` 或物理底盘。

2026-10-10 联调结果：map_server、global_costmap、local_costmap 均完成激活；
local_costmap 观察到约 3.5 Hz 输出。测试地图和 legacy 外参只用于接口回归，
不能替代正式场地地图、实测标定和障碍物验收。
