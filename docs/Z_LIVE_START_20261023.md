# ASUS 真实建图启动与使用（2026-10-23）

检查代码基线：`de088cb13ad78804f15310ae329759be527d026d`。
本次已直接启动真实 MID-360，不是 rosbag 回放。原车源码没有修改；未打开串口发送、未启动物理导航输出。

## 现在的实例

- ASUS：`ssh -p 2233 asus@192.168.1.164`。
- 工作区：`/home/asus/nav_2027_vehicle_config_164/z_migration_20261023`。
- ROS Domain：98。
- 雷达：192.168.1.152，主机雷达网口：192.168.1.50。
- 已启动：官方驱动、ZLIO、在线回环/图优化、sensor_reference、Foxglove。
- 尚未保存的地图目标：工作区下 `maps/live_20261023`。该目录保存时才创建。
- 日志、启动 PID 和观测报告：工作区下 `runtime_live_20261023/`。
- 后台启动辅助脚本：工作区下 `start_live.bash`。当前实例已经运行，请勿重复启动。

每个新 SSH 终端先执行：

```bash
export NAV_Z_WORKSPACE=/home/asus/nav_2027_vehicle_config_164/z_migration_20261023
cd "$NAV_Z_WORKSPACE/src/Adam_nav_2027"
source tools/z_environment.bash
```

## 实际启动检查

独立 CLI 采样：原始点云约10Hz，IMU约200Hz。启动后20秒观察收到3938条里程计、199帧去畸变点云、9次全图快照；包含订阅发现时间，计数不作为精密频率测量。
后端末状态 NORMAL、epoch=0；输入报告 continuous original inputs。之后约2分钟复查仍 NORMAL。
日志实际出现 `loop scan-to-map accepted`；这证明在线回环代码执行，不等于独立真值精度验收。

观察端源消息年龄：里程计中位1.57ms/最大328ms；去畸变云中位7.94ms/最大275ms；原始云中位101ms（时间戳位于扫描前部）。存在偶发停顿，不能声称实时延迟完全通过。
地图快照header是发布时刻，其年龄不能代表所有历史点的年龄，也不是Foxglove屏幕端延迟。

TF动态发布者只有 sensor_reference 和 backend_node，观察到 map→odom→front_mid360_imu，以及 odom→nav_reference、sensor_yaw_reference。
采样时ZLIO约55.9MiB RSS，后端约183.8MiB RSS；这是短时采样，不是长期内存上限。

证据：[20秒原始观察报告](evidence/z_localization_20261023/live_hardware_20s.json)。本次没有测试运动精度、断电恢复、地图保存或真实导航输出；没有重跑与启动无关的全量构建/单测。

## 在 Foxglove 看什么

1. WebSocket连接 `ws://192.168.1.164:8765`。
2. 导入仓库 `src/rm_nav_bringup/config/foxglove/z_localization_mapping.json`。
3. 3D固定坐标选择 map。
4. 首先只开启 `/backend_node/runtime_map`、`/backend_node/optimized_path` 和 `/backend_node/loop_edges`。

runtime_map是经过后端优化重建的历史累计图，约2秒刷新；不必用Foxglove额外长时间衰减叠加历史快照。
optimized_path是优化轨迹，loop_edges是接纳的回环约束。
要观察最新扫描，再单独打开 `/localization/lio_world_cloud`。
原始 `/livox/lidar` 与 `/lio/deskewed_odom_cloud` 都不是全历史地图。
旧布局中的 `/visualization/map_cloud.header` 没有数据时，换成 `/backend_node/runtime_map.header`；不要继续等旧地图发布器。

本次实际发现已连接的旧布局订阅多路大点云，bridge有outbox full提示。先关闭这些订阅，再用上述新布局。2秒的累计图刷新周期与当前扫描传输延迟应分别判断。

终端查看状态和日志：

```bash
bash tools/z_nav.bash status
# 另一个终端
tail -f "$NAV_Z_WORKSPACE/runtime_live_20261023/mapping.log"
```

建图模式 `/localization/healthy=false` 是预期：不授予导航许可。

## 完成一圈后怎么保存

停车后执行，保留后端运行：

```bash
bash tools/z_nav.bash save
ls -lh "$NAV_Z_WORKSPACE/maps/live_20261023"
```

成功后包含map.pcd、关键帧索引及原生关键帧/描述子文件。保存会停止前端；保存后不能继续这一轮扫描。失败请保留日志，不要直接删除旧图或覆盖同名目录。

当前后台mapping会话启动PID为4269，foxglove为4270，driver为3966，均独立进程组；**仅适用于本次未重启的会话**。需要结束本次建图时，先核对PID内容确为上述新工作区，再发SIGINT：

```bash
ps -p 4269,4270,3966 -o pid,args
# 确认仍是本次实例后；先保存再退出mapping
kill -INT -- -4269
# 若还需结束可视化和雷达
kill -INT -- -4270 -3966
```

重启后不能照抄历史PID。自行前台启动时直接Ctrl+C退出对应终端即可。
新一轮使用README中的driver、mapping、foxglove命令和不存在的新地图目录。

## 保存后怎么导航

原生点云地图还没有Nav2自由空间定义。先按[迁移文档](Z_LOCALIZATION_MIGRATION.md#从原生z地图到rm导航地图)导入draft，检查墙体/重影、地面高度和通行区域，生成检查后的占据栅格并发布local_site MapBundle。
不要把旧地图的自由空间蒙版套到新地图。

退出mapping后，保持driver和Foxglove，另开两个已加载环境的终端：

```bash
bash tools/z_nav.bash state
# 另一个终端：替换为本次检查并发布后的真实路径
bash tools/z_nav.bash navigation /absolute/published/map_bundle.json local_site
```

等待后端NORMAL且 `/localization/healthy=true`，才进入目标/规划验证。此入口仍不打开物理串口；本次启动通过不代表已经完成自动驾驶验收。
