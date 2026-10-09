# Foxglove 实时可视化

## 1. 启动 Bridge

ASUS 上执行：

```bash
cd /home/asus/nav_2027/rm_nav_v2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup foxglove_visualization.launch.py
```

默认监听 `0.0.0.0:8765`。Windows 上打开 Foxglove Studio，选择 **Open connection → Foxglove WebSocket**，
地址填写：

```text
ws://192.168.1.145:8765
```

如果 Wi-Fi 地址变化，在 ASUS 上用 `ip -4 addr show wlo1` 查询新地址。Bridge 只提供可视化数据，
不会启动底盘、串口、LIO 或导航节点。

## 2. Foxglove Layout 建议

添加一个 **3D Panel**，固定 `Frame` 为 `odom`（查看原始/局部定位）或 `map`（查看正式地图）。
打开 TF 显示，并加入以下图层：

| 可视化内容 | 话题 | Frame/用途 |
| --- | --- | --- |
| 原始 MID-360 点云 | `/livox/lidar` | `front_mid360`，检查原始字段和遮挡 |
| 质量门后的点云 | `/sensors/front_mid360/guarded_points` | 只有质量通过时发布 |
| 去畸变点云 | `/lio/deskewed_odom_cloud` | `odom`，用于检查 LIO 输出 |
| 实时局部子地图 | `/localization/odom_submap` | `odom`，实时建图/定位窗口 |
| 冻结地图输入 | `/localization/frozen_map` | `map`，需要定位链发布 |
| 可视化 PCD 地图 | `/visualization/map_cloud` | 启动参数指定的 `map_frame` |
| 原始 LIO 位姿 | `/lio/sensor_odometry` | `odom → front_mid360_imu` |
| 底盘状态 | `/state/chassis` | `odom → base_footprint` |
| Nav2 输出 | `/odom` | `odom → base_link` |
| LIO 轨迹 | `/visualization/lio_path` | `nav_msgs/Path`，有界 5000 点 |
| 底盘轨迹 | `/visualization/chassis_path` | `nav_msgs/Path`，有界 5000 点 |
| Nav2 轨迹 | `/visualization/nav_path` | `nav_msgs/Path`，有界 5000 点 |
| 优化轨迹/关键帧/回环 | `/visualization/optimization_graph` | `MarkerArray`，离线建图结果 |
| 地图版本状态 | `/visualization/map_bundle_status` | JSON `String`，版本、哈希、坐标系、发布资格 |
| 地图修正 | TF `map → odom` | 仅 MapOdomManager 发布 |
| 质量状态 | `/sensors/front_mid360/cloud_healthy` | 通过/拒绝 |
| 质量原因 | `/sensors/front_mid360/cloud_reason` | 退化、超时、字段错误原因 |

再添加 **Raw Messages** 或 **Plot** 面板观察：

- `/state/lio_healthy`、`/state/chassis_healthy`、`/localization/healthy`
- `/mapping/recorder_status`、`/mapping/keyframe_archive`、`/mapping/recorder_healthy`
- `/visualization/lio_path`、`/visualization/chassis_path`、`/visualization/nav_path`
- `/localization/status_reason`、`/localization/submap_reason`

建议保存两个 Layout：

1. `MID360 实时输入`：原始点云、质量门点云、IMU/健康状态、TF。
2. `建图与定位`：PCD 地图、局部子地图、去畸变点云、`map→odom→base_link` TF、状态原因。

## 3. 查看已生成地图

优化器输出的是二进制 XYZ PCD，坐标声明在文件头中。启动 Bridge 时指定：

```bash
ros2 launch rm_nav_bringup foxglove_visualization.launch.py \
  map_pcd:=/absolute/path/to/rebuilt_map.pcd \
  map_frame:=mapping_odom \
  graph_poses:=/absolute/path/to/optimized_poses.json \
  graph_loops:=/absolute/path/to/loop_edges.json \
  map_bundle:=/absolute/path/to/map_bundle.json
```

该 PCD 会以 transient-local `/visualization/map_cloud` 发布，Foxglove 连接后仍可收到。
它不自动变成正式 `map`，也不替代 MapBundle/官方坐标对齐。
如果同时指定图优化文件，`/visualization/optimization_graph` 会显示蓝色优化轨迹、黄色关键帧和红色已验收回环边。
该 MarkerArray 可直接放入 Foxglove 3D Panel，不需要手工解析 JSON。

可为离线结果生成显式地图清单：

```bash
python3 tools/create_map_bundle.py \
  --optimized-dir /absolute/optimized_output \
  --output /absolute/optimized_output/map_bundle.json
```

当前清单状态为 `draft_mapping_odom`，带 PCD、位姿、回环和输入哈希；
`publishable_for_frozen_localization` 明确为 false，完成官方坐标对齐、占据栅格和实测验证前不能启动正式冻结地图。

## 4. 实时建图查看

实时建图必须同时启动传感器、LIO、状态链和关键帧采集；Bridge 单独启动不会产生地图：

```bash
# 终端 1：官方 Livox 驱动和真实 MID-360
source /home/asus/nav_deps/livox_ws/install/setup.bash
ROS_DOMAIN_ID=104 ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2：按实测 CalibrationBundle 启动 LIO/状态/观测子地图
source /home/asus/nav_2027/rm_nav_v2/install/setup.bash
ROS_DOMAIN_ID=104 ros2 launch rm_nav_bringup local_state.launch.py \
  calibration_file:=/absolute/measured_calibration.yaml \
  lio_params_file:=/absolute/lio_params.yaml \
  enable_mid360_guard:=true

# 终端 3：显式存储根目录，开始原始关键帧归档
ROS_DOMAIN_ID=104 ros2 launch rm_nav_bringup mapping_capture.launch.py \
  archive_root:=/home/asus/nav_data/mapping_sessions \
  calibration_id:=measured-bundle-v1

# 终端 4：Foxglove Bridge
ROS_DOMAIN_ID=104 ros2 launch rm_nav_bringup foxglove_visualization.launch.py
```

在 Foxglove 中重点观察 `/livox/lidar` → `/sensors/front_mid360/guarded_points` →
`/lio/deskewed_odom_cloud` → `/localization/odom_submap` 的数据链。
`/mapping/keyframe_archive` 只是提交状态，不是点云；实时“地图效果”应使用
`/localization/odom_submap`。关键帧归档停止后，再运行离线优化器生成最终 PCD；
离线 PCD 不会自动覆盖实时子地图。

最近一次 40 s 实机采集已验证原始输入、局部子地图输入和 LIO 回放链；该次雷达没有形成足够大的可量化运动，
因此轨迹图可用性已验证，动态精度尚未验收。下一次采集应在 Foxglove 中同时观察三条 Path，
并记录明确的移动距离、旋转角度和起止位置。

## 5. 时间、Frame 和安全边界

- 连接 Foxglove 前确保电脑与 ASUS 在同一网络；有线雷达网口仍使用 `.50/32`，不能改回 `.50/24`。
- 播放 rosbag 时使用独立 `ROS_DOMAIN_ID`、`use_sim_time` 和 `/clock`；不要与物理驱动共用 domain。
- 没有实测标定时，不要把 `front_mid360` 直接当作 `base_link`；不要手工补 TF。
- Foxglove 显示数据不会证明定位精度；健康状态、源时间和 Frame 必须同时检查。
- 实时建图和控制输出隔离；不要为了 Foxglove 直接打开 `/nav/cmd_vel_safe` 到物理串口。
