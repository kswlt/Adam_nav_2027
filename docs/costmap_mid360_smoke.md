# MID-360 Costmap 联调

测试地图与完整联调启动：

```bash
ros2 launch rm_nav_bringup costmap_mid360_smoke.launch.py \
  profile:=TEST_ONLY map_yaml:=/tmp/smoke_map.yaml
```

`TEST_ONLY` 只验证地图、TF、LaserScan 和 Costmap 消息链，使用显式静态测试 TF，
禁止进入生产 Domain。它仍需要外部 `/scan` 或 guarded PointCloud2 输入。

`LEGACY_DEBUG` 依赖外部运行的 Livox/Point-LIO `/lio/sensor_odometry`，并使用原版静态
外参联调；它不会发布测试 `map→odom`。`PRODUCTION` 会直接拒绝，正式系统必须使用
实测 CalibrationBundle、真实状态链和 MapOdomManager。

2026-10-10 联调结果：map_server、global_costmap、local_costmap 均完成激活；
local_costmap 观察到约 3.5 Hz 输出。测试地图和 legacy 外参只用于接口回归，
不能替代正式场地地图、实测标定和障碍物验收。

Costmap footprint 当前复用了原版 Adam reality 配置的 0.2 m 方形；该值只是兼容基线，
必须用底盘实测长宽、旋转包络和制动距离重新确认后才能作为正式参数。
