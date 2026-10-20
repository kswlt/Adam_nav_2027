# Adam_nav_2027

RoboMaster 全向底盘导航框架，环境为 Ubuntu 24.04、ROS 2 Jazzy、Nav2 Jazzy。
雷达前端使用 **small_point_lio**，电控通信保留 **my_serial_py** 原协议。
本页按实际操作顺序说明如何启动、查看 Foxglove、保存地图以及进入导航。

2026-10-20 已在 ASUS 验证真实 MID-360、本地状态、TF、滚动子图、建图预览及关键帧录制。
全 14 包构建通过；回归结果为 49 tests、0 errors、0 failures、1 skipped（可选 libscrc 检查）。
这些是历史实测结果，不代表节点一直运行：本次文档更新检查时，8765 没有监听，也未发现 Livox、small_point_lio、Foxglove 进程。
**正式场地地图、实测 footprint、yaw/下位机 watchdog 和实际车辆运动仍需验收。**

## 1. 当前电脑与目录

| 项目 | 配置 |
| --- | --- |
| SSH | `ssh -p 2233 asus@192.168.1.164` |
| 独立部署目录 | `/home/asus/nav_2027_vehicle_config_164` |
| 新源码及 build/install | `/home/asus/nav_2027_vehicle_config_164/releases/20261020` |
| 运行配置、日志、关键帧 | `/home/asus/nav_2027_vehicle_config_164/runtime_20261020` |
| Foxglove | `ws://192.168.1.164:8765` |
| ROS Domain | `97`，由环境脚本设置 |
| 雷达 / 有线网卡 | `192.168.1.152` / `192.168.1.50/32`，独立有线路由 |
| 当前雷达安装 | 固定正装，车体 x=0、y=+0.08、z=+0.20 m |

本套部署与原车代码隔离。不要覆盖原工作区，也不要使用历史报告中的 `.145` 地址启动本套系统。
安装位移只在 TF 应用一次：新驱动使用 `runtime_20261020/mid360_identity.json`，不要换回带安装位移的旧 JSON。
当前标定是用户提供安装位置加工厂内部外参，未冒充实测 verified。

## 2. 重启后怎么启动

先登录 ASUS，检查已有节点与端口，避免重复启动：

```bash
source /home/asus/nav_2027_vehicle_config_164/remote_env_20261020.bash
ros2 node list
ss -ltn 'sport = :8765'
```

已有本套实例时直接使用；只有部分节点时检查对应日志，只补缺失环节。
没有实例时，按下面顺序在 **三个独立 SSH 终端** 启动，先确认前一步成功再继续。

```bash
# 终端 1：MID-360 驱动
cd /home/asus/nav_2027_vehicle_config_164
bash start_driver_20261020.bash
```

```bash
# 终端 2：质量门、small_point_lio、resolver、EKF、TF、滚动子图
cd /home/asus/nav_2027_vehicle_config_164
bash start_state_20261020.bash
```

```bash
# 终端 3：Foxglove Bridge、历史轨迹、实时累积预览
cd /home/asus/nav_2027_vehicle_config_164
bash start_foxglove_20261020.bash
```

对应终端 Ctrl+C 停止；不要执行 `pkill ros2`，避免停止原车程序。
这些脚本不启动电控串口，不发送物理底盘运动命令。原文见 [部署脚本](tools/deployment/20261020/README.md)。

另开终端检查输出，`hz` 和 `tf2_echo` 持续运行，观察后 Ctrl+C 再执行下一条：

```bash
source /home/asus/nav_2027_vehicle_config_164/remote_env_20261020.bash
ros2 topic hz /livox/lidar
ros2 topic hz /lio/deskewed_odom_cloud
ros2 topic echo /state/lio_reason --once
ros2 topic echo /visualization/live_map_status --once
ros2 run tf2_ros tf2_echo odom front_mid360
```

预览配置为 **1 Hz、0.15 m 体素、最多 50000 个点**；原始雷达约 10 Hz，二者不是同一更新率。
只看到话题名，没有持续新消息，不算启动成功。

## 3. Foxglove 怎么连接和配置

1. 在自己的电脑打开 Foxglove，选择 Foxglove WebSocket，填写 `ws://192.168.1.164:8765`。
2. 布局菜单选择 **Import from file / 从文件导入**，导入 [mapping_low_latency.json](src/rm_nav_bringup/config/foxglove/mapping_low_latency.json)。JSON 需先放到运行 Foxglove 的电脑。
3. 打开 3D 面板设置，建图时把 **Fixed frame、Display frame 都设为 `odom`**。启用 `/visualization/live_map_preview`，在 Transforms 中开启坐标轴、标签、连线。
4. 要看瞬时扫描，临时启用 `/livox/lidar` 或 `/lio/deskewed_odom_cloud`。尽量一次只显示一种高带宽点云。
5. 正式导航阶段可导入 [navigation_tdt.json](src/rm_nav_bringup/config/foxglove/navigation_tdt.json)，只有真实全局定位建立 `map → odom` 后才使用 `map` 世界参考。

导入方法见 [Foxglove 官方布局说明](https://docs.foxglove.dev/docs/visualization/layouts)。
仓库布局 JSON 已检查，具体桌面客户端的导入效果仍需现场确认。

### 点云、地图和轨迹各是什么意思

| 话题 | 看到什么、用来判断什么 |
| --- | --- |
| `/livox/lidar` | 雷达当前原始扫描，雷达坐标；看接收和遮挡，不是完整地图 |
| `/sensors/front_mid360/guarded_points` | 质量门通过后供 LIO 使用的输入云，不是去畸变地图 |
| `/lio/deskewed_odom_cloud` | LIO 去畸变并转到 odom 的单次扫描；检查运动时墙面重影、撕裂 |
| `/localization/odom_submap` | 近期观测组成的滚动子图，供配准使用；旧区域会退出窗口 |
| `/visualization/live_map_preview` | odom 下实时体素累积预览；移动后已扫描区域增加，满容量后淘汰旧体素 |
| `/visualization/map_cloud` | 显式加载离线 PCD 后的静态地图；默认启动没有加载它 |
| `/map` | 正式地图服务器发布的二维占据栅格，不是彩色三维点云 |
| `/global_costmap/costmap`、`/local_costmap/costmap` | 全局/局部障碍、膨胀和通行代价；导航环境层启动后才有 |
| `/visualization/lio_path` | LIO 传感器 IMU 原点的已走历史轨迹 |
| `/visualization/chassis_path` | chassis 车体原点的已走估计轨迹 |
| `/visualization/nav_path` | `/odom` 中 Nav2 参考点的已走估计轨迹，**不是未来路线** |
| `/plan` | Nav2 全局规划的未来路线，没有导航任务时可能没有 |
| `/visualization/tdt_path` | TDT 生成的未来参考轨迹，可与实际轨迹比较 |
| `/visualization/optimization_graph` | 离线优化关键帧、轨迹及接纳的回环约束，需另行加载结果 |

三条历史 Path 分别来自 `/lio/sensor_odometry`、`/state/chassis`、`/odom`，各最多保存 5000 个点。
它们参考点不同，重叠或偏移要结合 TF 判断，不能把差值直接当作真值误差。
**实时预览不等于已经保存、回环优化或正式对齐的 MapBundle。**

### 截图里的彩色点和蓝网格

点云颜色由该话题的 **Color mode、Color field、色带范围** 决定：选择 `z` 表示高度，选择 `intensity` 表示回波强度。
红黄蓝本身不代表危险或可通行。当前预览只有 XYZ，没有 intensity，应选 `z` 或固定颜色。
仅凭截图不能确定当时启用了哪个云、哪个着色字段，应查看 3D 设置中已启用的话题。
蓝色参考网格只是显示标尺，不是占据地图或可行驶区域；画面倾斜也可能来自观察相机角度。
设置含义见 [官方 3D 文档](https://docs.foxglove.dev/docs/visualization/panels/3d)。

优化图的颜色由本工程固定：青蓝线=优化后轨迹，黄色球=关键帧，红线=接纳的回环约束。
算法接纳回环不等于已经由定位真值证明正确。

### TF 坐标系和坐标轴

当前固定安装链：

```text
odom → base_footprint → chassis → front_mid360_imu → front_mid360
                              └→ base_link
```

| 坐标系/变换 | 含义 |
| --- | --- |
| `odom` | 本次运行的连续本地参考，不是官方场地坐标 |
| `odom → base_footprint` | 状态估计链唯一发布的平面 x/y/yaw |
| `base_footprint → chassis` | 保留车体高度、roll、pitch，完整车体状态没有被全部压成二维 |
| `chassis → base_link` | 当前 Nav2 参考别名，不要另发冲突父节点 |
| `front_mid360_imu` | 雷达内部 IMU 原点，与 LiDAR 测量原点有偏移 |
| `front_mid360` | LiDAR 点云坐标；通过安装位置及工厂内部外参连接车体 |
| `map → odom` | 正式全局定位通过后由 MapOdomManager 发布，纯建图观察阶段没有是正常的 |

坐标轴通常 x 红、y 绿、z 蓝；标签表示 frame，连线表示父子关系。
当前是固定安装，没有 `big_gimbal_yaw` 不是漏发；动态云台属于另一安装 profile。
红色感叹号需查看具体错误、消息 frame 和该消息时刻的 TF，不能用假的静态 map→odom 消除警告。

### 状态面板和曲线

Raw Messages 面板建议分别选择下列话题：

| 话题 | 应看什么 |
| --- | --- |
| `/visualization/live_map_status` | JSON 的 `points`、`accepted_scans`、`input_stale`、`reason`，断流应标记 stale |
| `/state/lio_reason` | LIO 质量、时间和输入失败原因 |
| `/state/chassis_healthy` | 车体估计健康；true 不代表轮速或制动已验收 |
| `/localization/submap_healthy`、`/localization/submap_reason` | 子图可用性和原因，不等于正式全局定位健康 |
| `/localization/status_reason` | 正式地图定位状态，对应链启动后才有 |
| `/mapping/keyframe_archive` | 最近保存的关键帧路径字符串，不是点云；静止时不会持续产生新帧 |
| `/nav/task_status`、`/nav/motion_gate_reason` | 任务和运动许可/拒绝原因，导航启动后才有 |
| `/nav/tdt/report` | TDT 求解与合法性报告，该后端启动后才有 |

Plot 需要添加数值字段，例如：

```text
/odom.pose.pose.position.x
/odom.pose.pose.position.y
/odom.twist.twist.linear.x
/odom.twist.twist.linear.y
/odom.twist.twist.angular.z
/livox/imu.angular_velocity.z
```

位置是米、线速度 m/s、角速度 rad/s；odom 速度是状态估计，不是实测轮速。
本入口的原始 IMU 加速度按 g 处理，不能直接当作 m/s²。
String/JSON 状态先用 Raw Messages；图表显示“点击来添加数据系列”只是尚未选数值曲线。

### 针对截图里的两个空面板

1. 左下 `/visualization/map_cloud.header` 改成 `/visualization/live_map_status` 看建图状态，或 `/visualization/live_map_preview.header` 看源时间和 frame。3D 必须选完整点云话题，不选 `.header`。默认没加载离线 PCD，所以旧 map_cloud 面板等不到消息。
2. 右下 Plot 添加上面的 x/y 或速度字段；也可删除空图表扩大 3D。
3. 点开顶部“问题 2”及红色连接提示读取实际错误，截图不能证明错误类型。旧点云可能停留在画面中，要同时确认端口监听和消息时间持续更新。
4. 左侧 Hz 是客户端观测到的接收频率；`—` 可能未订阅或暂未收到消息，不能单独证明节点离线。

### 卡顿与延迟

先只打开累积预览、TF、两条历史轨迹，关闭原始云、去畸变云、子图与完整 PCD；点云 Decay time 设 0，避免客户端额外累积。
预览 1 Hz 有约一个周期的更新间隔，但持续落后四五秒还需查网络、渲染、TF 时间和连接状态。
当前部署脚本 bridge 缓冲 1 MiB，不能保证消除所有延迟。ASUS 与 Windows 先前测得约 8 小时时钟差，
跨机接收时间减 header 不能直接当延迟，须先统一实际时钟。
更多逐步排查见 [Foxglove 详细指南](docs/foxglove_visualization.md)。

## 4. 在线回环优化建图

现在可在录制时接入在线回环：后台检测回环、GTSAM 修正历史关键帧，按新位姿重建并替换地图。
已有 odom 累积预览仍保留，优化地图在独立 `mapping_odom` 参考下查看。

```bash
cd /home/asus/nav_2027_vehicle_config_164
bash start_online_mapping_20261020.bash /绝对路径/当前的session
```

Foxglove 导入 [mapping_online.json](src/rm_nav_bringup/config/foxglove/mapping_online.json)，
启用 `/mapping/optimized_map_preview`、`/mapping/optimized_path`、`/mapping/loop_graph`；
Raw Messages 看 `/mapping/online_status` 的回环数量、版本、积压与原因。
默认 500 帧、0.15 m 体素、最多 50 万地图点，Wi-Fi 预览最多 5 万点；不是无限规模地图。
完整操作、资源预算和验收见 [在线建图指南](docs/ONLINE_MAPPING.md)。

## 5. 怎么保存建图、查看离线地图

只启动可视化不会保存最终地图。雷达和状态链运行时另开第四个终端：

```bash
cd /home/asus/nav_2027_vehicle_config_164
bash start_mapping_20261020.bash
```

关键帧在 `runtime_20261020/keyframes/session_*`，每次启动创建新 session。
按运动阈值录制，所以静止时只有少量帧正常。当前单段上限 500 帧，超限应换新 session；未实现自动跨段全局拼接。
结束先 Ctrl+C 停 recorder，再优化。保留原始 CDR 和失败证据。
下面占位路径须换成实际路径，输出使用新目录：

```bash
source /home/asus/nav_2027_vehicle_config_164/remote_env_20261020.bash
cd /home/asus/nav_2027_vehicle_config_164/releases/20261020
ros2 run rm_nav_mapping offline_graph_optimizer /绝对路径/session /绝对路径/optimized --enable-loops
python3 tools/create_map_bundle.py --optimized-dir /绝对路径/optimized --output /绝对路径/optimized/draft.json
```

复用已有 bridge，只启动 PCD 发布器；PCD 文件名以实际优化输出为准：

```bash
ros2 run rm_nav_bringup pcd_map_publisher --pcd /绝对路径/optimized/rebuilt_map.pcd \
  --frame mapping_odom --topic /visualization/map_cloud
```

另建 3D 面板，Fixed/Display frame 均为 `mapping_odom`，开启 map_cloud。
不要把未经对齐的 mapping_odom 硬连到实时 odom。查看回环图：

```bash
ros2 run rm_nav_bringup foxglove_graph_publisher \
  --poses /绝对路径/optimized/optimized_poses.json \
  --loops /绝对路径/optimized/loop_edges.json --frame mapping_odom
```

正式发布还需官方场地对应点、可靠高程与自由空间掩码，然后生成对齐 PCD、occupancy.pgm/map.yaml、terrain_reference。
格式与命令见 [完整软件指南第 4 节](docs/SOFTWARE_USAGE_20261013.md#4-建图对齐发布)。
无官方资料则保持 draft；terrain 当前只是观测高度参考，不是已验收可通行地形。

## 6. 怎么进入正式导航

需要本场地已发布且 navigation-ready 的 MapBundle、实测底盘轮廓、真实全局定位与健康输入。
下例仅启动软件链，物理输出与串口关闭；占位符替换后才能执行：

```bash
ros2 launch rm_nav_bringup vehicle_navigation.launch.py \
  map_bundle:=/绝对路径/published_map/map_bundle.json \
  footprint:='实际测量的 [[x,y],...]' \
  control_backend:=direct_omni enable_physical_output:=false start_serial:=false
```

复用已有本地状态链，不再启动另一套 LIO。全局配准通过后才建立 map→odom。
初始提示 `/initialpose` 使用 map 下 base_footprint 位姿，不能靠提示直接获得健康。
任务入口 `/nav/submit_goal`（`rm_nav_interfaces/srv/SubmitGoal`）；`/clicked_point` 只是点击坐标，不等于导航任务。
Supervisor 管理许可，直接发送 Nav2 action 不会自动获得任务所有权。

当前协议无真实速度/停车反馈。direct_omni 可显式配置为速度前馈；ros2_control 后端需真实 `/hardware/drive_feedback` 才能激活。
两后端互斥，my_serial_py 为唯一串口 owner。物理联调条件及参数见 [完整指南第 5 节](docs/SOFTWARE_USAGE_20261013.md#5-定位与导航)。
不要为查看地图开启物理运动输出。

## 7. 框架架构和代码职责

```text
MID-360 → Livox → 质量门 → small_point_lio
                            ├→ 去畸变云 → 滚动子图 / 实时预览
                            └→ 位姿 → SE3 resolver → robot_localization → 状态/TF/odom
原始关键帧 → KISS/GICP 回环 → GTSAM → draft MapBundle → 官方对齐/地图发布
正式地图 + 观测子图 → GICP 定位 → MapOdomManager → map→odom
地图/障碍 → Nav2 → TDT → Omni PID/YawManager → 监管许可
                                           └→ direct_omni 或 ros2_control → my_serial_py
```

| 包/目录 | 职责 |
| --- | --- |
| `rm_nav_bringup` | launch、参数、Foxglove 发布器与布局 |
| `rm_nav_sensors`、`rm_nav_frames` | 原始质量检查、标定、TF 与参考点解析 |
| `rm_nav_localization` | 状态桥接、子图、全局定位、map/odom 管理 |
| `rm_nav_registration`、`rm_nav_mapping` | 配准组件、关键帧、回环、图优化 |
| `rm_nav_perception` | 导航环境与障碍输入 |
| `rm_nav_planning`、`rm_nav_control` | TDT、轨迹验证、Omni PID/YawManager、监管与命令门 |
| `rm_nav_hardware`、`my_serial_py` | HardwareInterface、反馈/租约、原协议通信 |
| `rm_nav_interfaces`、`pb_rm_interfaces` | 消息/服务 |
| `rm_nav_sim`、`tools` | TEST_ONLY fixture、smoke、回放和验收 |

动态云台与当前固定雷达是不同安装 profile，不伪造编码器。
接口见 [架构契约](docs/architecture_contract.md)，完成度见 [阶段矩阵](docs/stage_completion_matrix.md)。

## 8. 构建、测试、故障排查

在已有 ASUS 部署中，环境脚本提供依赖和私有消息 ABI overlay：

```bash
source /home/asus/nav_2027_vehicle_config_164/remote_env_20261020.bash
cd /home/asus/nav_2027_vehicle_config_164/releases/20261020
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
```

全新环境依赖/构建见 [完整软件指南](docs/SOFTWARE_USAGE_20261013.md)，仅 clone 本仓库不会自动安装所有依赖。
完整软件验收入口 `bash tools/run_software_acceptance.bash`；理想底盘及合成测试不证明实际车辆运动。

| 现象 | 优先检查 |
| --- | --- |
| Foxglove 连不上 | SSH 网络、8765 监听、bridge 终端 |
| 原始云有，LIO 没有 | guarded_points、IMU、lio_reason 和状态终端 |
| TF 红色 | 参考 frame、消息 frame、消息时刻的 TF 和源健康 |
| map_cloud 没消息 | 是否加载 PCD；当前建图使用 live_map_preview |
| 有图但没有保存地图 | recorder 是否启动、是否停止后离线优化 |
| map/costmap/plan 没有 | 正式导航、合格地图、定位和任务是否启动通过 |
| Fast-CDR undefined symbol | 是否加载 remote_env 中同版本私有消息 overlay，不要全局覆盖 ROS 库 |

进一步阅读：[现场实测报告](docs/ASUS_LIVE_20261020.md)、[交接](docs/HANDOFF.md)、
[审计](docs/IMPLEMENTATION_AUDIT.md)、[导航验收](docs/NAVIGATION_INTEGRATION_ACCEPTANCE.md)、
[真实数据验收](docs/PUBLIC_MID360_ACCEPTANCE.md)、[Foxglove 详细指南](docs/foxglove_visualization.md)。
