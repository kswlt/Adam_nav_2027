# Foxglove 使用与排查（2026-10-20）

当前操作入口与完整话题表见 [README 第 2、3 节](../README.md#3-foxglove-怎么连接和配置)。
当前车载地址为 `ws://192.168.1.164:8765`，SSH 为 `asus@192.168.1.164:2233`，运行 Domain=97。
历史 `.145` 主机与旧路径不用于本次部署。

## 推荐布局

| 用途 | 布局 | 世界参考 | 默认重点 |
| --- | --- | --- | --- |
| 真实雷达/实时建图观察 | [mapping_low_latency.json](../src/rm_nav_bringup/config/foxglove/mapping_low_latency.json) | odom | 累积预览、历史轨迹、TF、原始状态 |
| 正式地图定位与导航 | [navigation_tdt.json](../src/rm_nav_bringup/config/foxglove/navigation_tdt.json) | map | 正式地图、规划/TDT 路线、代价图、监管状态 |
| 离线优化地图 | 新建独立 3D 面板 | mapping_odom | PCD 与优化图，不硬叠到实时 odom |

从布局菜单 Import from file 导入 JSON，具体步骤见 [官方布局说明](https://docs.foxglove.dev/docs/visualization/layouts)。
启动命令见 README；不要为增加一个面板再启动第二个占用 8765 的 bridge。
导航布局不能自动生成 map→odom、地图、costmap 或任务；必须先有真实对应生产链。

## 点云显示建议

调试雷达选 `/livox/lidar`；看 LIO 单帧效果选 `/lio/deskewed_odom_cloud`；
看近期配准观测选 `/localization/odom_submap`；看实时累积效果选 `/visualization/live_map_preview`。
四者是不同阶段，不要全部启用后将密集重叠误认为地图更完整。

预览默认 1 Hz、0.15 m、50000 体素点。体素按最近更新维护，容量满后淘汰旧体素，
所以已经扫描很远的区域消失可能是容量策略，不代表原始关键帧丢失。
预览只包含 x/y/z，没有 intensity。用 Color map + z 看高度，或者 Flat 设置固定颜色。
点大小可从 2 像素试起；Decay time 设 0，避免客户端再次累计整段历史。
颜色字段、色带和显示时刻说明见 [官方 3D 文档](https://docs.foxglove.dev/docs/visualization/panels/3d)。

蓝色参考网格不是 occupancy 或 costmap，彩色点也不是障碍等级。
优化图例则由工程固定：青蓝轨迹、黄球关键帧、红色已接纳回环边。

## 如何判断是真实时，还是停留在旧画面

1. 确认顶部连接状态，点开 Issues/问题，读取每一项完整错误。
2. Raw Messages 选择 `/visualization/live_map_preview.header`，观察 stamp 持续增加。
3. 选择 `/visualization/live_map_status`，观察 accepted_scans 增加、input_stale=false；points 不一定每帧增加。
4. 观察实际源话题与 TF。源停止后，旧点云可能仍留在画面，轨迹也可能仍显示最后一次状态。

预览在没有新点云时不重新发布并伪造新时间；状态会标记 input_stale。
`/visualization/map_cloud` 是可选静态 PCD，有些离线可视化消息时间为零，不适合用于判断实时输入延迟。
`/mapping/keyframe_archive` 是保存事件字符串，静止时不会连续更新，不能用它检测雷达帧率。

## TF 红色或点云不出现

3D 需要在点云源时间找到源 frame 到世界参考的变换，只有连线名字而缺对应时刻数据仍可能报错。
依次检查：

- 建图模式 Fixed/Display frame=odom，离线结果=mapping_odom，正式导航成功后=map。
- 点云 header.frame_id 是否属于正在运行的那套链，而非另一 ROS Domain 或旧回放。
- `/tf` 的动态变换是否持续，`/tf_static` 的固定安装是否收到，是否有重复 authority。
- 健康原因与时间是否异常。不可用静态 map→odom 代替未成功的全局定位。

在 ASUS 加载本次环境后执行，每条持续命令观察后 Ctrl+C：

```bash
source /home/asus/nav_2027_vehicle_config_164/remote_env_20261020.bash
ros2 run tf2_ros tf2_echo odom front_mid360
ros2 topic echo /state/lio_reason --once
ros2 topic echo /localization/submap_reason --once
ros2 topic info /tf --verbose
```

当前固定安装无动态云台 frame；IMU 原点、LiDAR 原点和 chassis 是不同参考点，不能直接相互改名。

## 延迟四五秒的排查顺序

1. 关闭高带宽云，只保留预览、TF、历史轨迹，检查是否恢复；CPU/GPU 渲染与网络都可能积压。
2. 检查预览状态和源时间是否更新；1 Hz 预览本身不会像约 10 Hz 单帧扫描那样连续。
3. 检查 bridge 终端是否退出或报缓冲/连接错误，ASUS 执行 `ss -ltn 'sport = :8765'`。
4. 检查 ASUS 本机源时间年龄，再核对两台机器实际时钟。先前跨机约 8 小时差不能解释为网络耗时，也不能忽略其对显示时间的影响。
5. 如需调整 bridge/预览参数，先停止自己所属的 Foxglove 启动实例再改，不能重复启动争用端口；不要停止原车其他 ROS 进程。

本次专用脚本缓冲 1 MiB；通用 launch 默认 4 MiB，不能把两者混为同一运行参数。
之前同机源年龄 p95（LIO 约 10.45 ms、子图约 96.41 ms、预览约 62.49 ms）仅是特定采样窗口，
不是桌面 UI 延迟承诺。实际 GUI 延迟还需在同一时间基准下观测。

## Plot 与状态面板

Plot 添加数值字段，如 `/odom.pose.pose.position.x`、`/odom.twist.twist.linear.y`、`/livox/imu.angular_velocity.z`。
字段路径用法见 [官方 Plot 文档](https://docs.foxglove.dev/docs/visualization/panels/plot)。
String/JSON 诊断用 Raw Messages，Bool 可用 Raw Messages 或 State Transitions。
不要把整条 Odometry 消息或 String 直接当一条数值曲线。

`/visualization/nav_path` 是历史轨迹；`/plan` 与 `/visualization/tdt_path` 才是未来参考。
速度来自状态估计时不是轮速真值。`submap_healthy=true` 也不等于正式地图定位或实车制动已经通过。

## 保存、回放与现场边界

建图先启动 recorder，再结束录制并离线优化；PCD、关键帧和最终 MapBundle 的区别及命令见 README 第 4 节。
观看 Foxglove 不会自动保存最终地图。离线优化结果发布器复用 bridge，显示参考为 mapping_odom。
rosbag 回放使用独立 ROS Domain、use_sim_time 和 /clock，不与真车驱动混在 Domain 97。
不要为消除显示告警开启物理控制或伪造 TF、编码器、速度和停车反馈。
