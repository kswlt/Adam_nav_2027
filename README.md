# Adam_nav_2027

**最新交付审查（2026-10-23晚）：** 当前是开发联调版，新Z链尚未完成实车自主导航验收。
[代码缺口、模块状态和UI完整操作说明](docs/CODE_DELIVERY_REVIEW_20261023.md)。
已发布本地场地地图；仍存在偶发输入延迟、车辆几何待校核、任务UI及整车验收缺口。
操作台“导航”是影子导航，不发送物理运动指令。以下“已启动”均是历史试验记录，不代表当前进程在线。

RoboMaster 全向哨兵导航框架，运行于 **Ubuntu 24.04 / ROS 2 Jazzy / Nav2 Jazzy**。

**当前定位方案已迁移到 [mozihe/z-localization](https://github.com/mozihe/z-localization)**：ZLIO 前端处理雷达/IMU，Z 后端负责在线回环、历史轨迹优化、地图更新和先验地图定位。原 `my_serial_py` 通信协议保留。旧 small_point_lio/KISS 定位链只保留作显式历史回归，不是默认定位入口。

完整迁移说明、依赖版本、测试证据和限制见 [Z-Localization 迁移记录](docs/Z_LOCALIZATION_MIGRATION.md)。**代码迁移不等于实车导航精度验收通过。**

**2026-10-23 已在 ASUS 实际启动真实 MID-360 + ZLIO + 在线回环建图 + Foxglove。** 当前连接 `ws://192.168.1.164:8765`；累计地图看 `/backend_node/runtime_map`，固定坐标选 `map`。本次后台实例、实测结果、具体保存路径和退出命令见 [现场启动与使用指南](docs/Z_LIVE_START_20261023.md)。本次没有启动底盘运动输出。

**建图/保存/导航操作UI和ASUS交互终端**见 [操作台指南](docs/Z_OPERATOR_UI.md)。桌面“新导航框架/Foxglove操作台”已准备组合布局与连接入口。后端和真实PTY测试通过；Foxglove原生扩展的实际加载尚未完成屏幕验证。累计图曾因intensity全0显示紫色，新布局改为按高度着色。

操作台现已显示**俯视地图圈选、正式本地地图发布、影子目标输入**。本次场地地图已经人工确认并发布local_site MapBundle；新地图仍需重新审核。现场新原始数据加载已保存Z地图的定位回放通过；导航整链尚未验收，轴参考连续性保护仍需处理。步骤与实测见 [地图检查和导航交付](docs/Z_MAP_REVIEW_20261023.md)。

## 先区分三个操作

| 要做的事 | 启动什么 | 得到什么 |
|---|---|---|
| 建图 | 雷达驱动 + mapping | ZLIO、在线回环/图优化、累计地图 |
| 保存地图 | 保持后端运行，再执行 save | 原生Z关键帧地图及优化后的 map.pcd |
| 用地图导航 | state + navigation | 先验地图定位 + Nav2；默认不启动物理串口 |

当前测试车：MID-360 随 yaw 转动，雷达偏心安装。导航参考点使用已批准的约15.34cm偏心拟合；`nav_reference` 是导航计算参考，不是底盘真实朝向。不要把雷达 yaw 当成底盘 yaw，也不要复制其他车辆的标定参数。

## 连接 ASUS

```powershell
ssh -p 2233 asus@192.168.1.164
```

用户名 `asus`，密码使用你现有的 ASUS 登录密码。

此次隔离目录：

```text
/home/asus/nav_2027_vehicle_config_164/z_migration_20261023/
├── src/z-localization/       固定版本的完整上游源码
├── src/Adam_nav_2027/        本框架
├── build/、install/、log/    此次独立构建
└── build_full.log、test_result.log
```

`/home/asus/z-localization-jazzy` 和既有车载导航属于其他现有实例，本次不覆盖、不停止。
原 `20261022_fit153_console` 菜单运行的是旧 release；**新流程使用下面的 z_nav 命令。**

## 每个终端先加载环境

在 ASUS 上：

```bash
export NAV_Z_WORKSPACE=/home/asus/nav_2027_vehicle_config_164/z_migration_20261023
cd "$NAV_Z_WORKSPACE/src/Adam_nav_2027"
source tools/z_environment.bash
```

其他电脑设置自己的 `NAV_Z_WORKSPACE` 和 `NAV_LIVOX_INSTALL`。安装/构建步骤见 [迁移记录](docs/Z_LOCALIZATION_MIGRATION.md#源码构建与许可证)。

此环境默认 **ROS_DOMAIN_ID=98**，与旧实例隔离；如确需其他Domain，先设置 `NAV_Z_DOMAIN_ID`。驱动、状态、建图、导航、Foxglove必须使用同一值。

雷达驱动单独启动，并确保 ROS_DOMAIN_ID 一致。**不要同时开启两套驱动抢占同一个 MID-360。**下面的工具不自动接管其他实例、不配置网卡、不打开物理串口。

确认其他驱动已经由其操作者停止后，使用你实际雷达/主机IP的JSON启动：

```bash
bash tools/z_nav.bash driver /absolute/mid360_identity.json
```

此入口固定使用本框架已经检查过的官方PointCloud2格式。其他实例的CustomMsg不能直接接到本入口。

## 1. 建图

先让底盘和云台都停稳，完成初始化后再缓慢移动、转动。新终端：

```bash
bash tools/z_nav.bash mapping "$HOME/adam_z_maps/session_001"
```

每次使用不存在的新输出目录。此命令启动完整状态链和Z在线后端；不需要再开旧 capture/online 组件。

另开终端连接可视化：

```bash
bash tools/z_nav.bash foxglove
```

Foxglove 新建 WebSocket 连接 `ws://192.168.1.164:8765`，导入布局：
`src/rm_nav_bringup/config/foxglove/z_localization_mapping.json`。
如果8765已有其他实例占用，给本次 bridge 传 `port:=8766`，连接对应端口。

## 2. 查看建图与定位

3D面板显示坐标选择 **map**。

| 话题 | 含义 | 建议 |
|---|---|---|
| `/backend_node/runtime_map` | 全部运行关键帧按优化位姿重建的累计地图 | 建图主要看这个，默认约2秒更新 |
| `/backend_node/optimized_path` | 回环/图优化后的历史轨迹 | 开启 |
| `/backend_node/loop_edges` | 后端接纳的回环边 | 开启 |
| `/backend_node/odom_path` | 原始里程计轨迹对照 | 排查漂移时开启 |
| `/backend_node/prior_map` | 定位模式加载的固定先验图 | 导航时开启 |
| `/localization/lio_world_cloud` | 全局定位下的当前扫描 | 检查扫描是否贴合先验图 |
| `/lio/deskewed_odom_cloud` | ZLIO去畸变后、转换到odom的当前观测 | 不是整张地图 |
| `/livox/lidar` | 原始扫描 | 只检查输入时开，避免重复显示卡顿 |
| `/tf`、`/tf_static` | 坐标关系与传感器安装外参 | 开启变换/坐标轴 |
| `/backend_node/localization_state` | NORMAL/LOST等状态、epoch及原因 | 原始消息面板 |
| `/localization/healthy` | 给导航使用的真实定位健康许可 | 建图模式为false是预期行为 |

**累计地图每2秒刷新与实时扫描延迟是两回事。**多路大点云一起开会明显增加带宽和渲染负担；日常先只开累计地图、优化轨迹和TF。
旧 `/visualization/live_map_preview` 只是odom累计预览，不代表回环已经修正历史地图。

终端持续看消息：

```bash
bash tools/z_nav.bash status
ros2 topic echo /lio/estimator_reason
ros2 topic echo /state/sensor_reference_reason
ros2 topic hz /lio/sensor_odometry
```

## 3. 保存地图

停车，保持建图终端和后端运行。在另一个已加载环境的终端执行：

```bash
bash tools/z_nav.bash save
```

脚本先排空并停止前端，再请求后端保存；任一步失败会直接报错。
保存位置是启动mapping时填写的目录，包含 `keyframes.txt`、`clouds/`、描述子及 `map.pcd`。
**保存后前端已停止**，下次建图/导航先退出旧mapping，再重启状态链。

## 4. 地图检查与发布

原生Z地图不等于Nav2占据栅格。必须检查墙体重影、地面/高度、闭环、障碍物和自由空间，生成RM MapBundle。

```bash
python3 tools/import_z_map.py --native-bundle "$HOME/adam_z_maps/session_001" \
  --output "$HOME/adam_z_maps/draft_001" \
  --calibration-id vehicle164_yaw_axis_fit153cm_debug_v3
```

随后使用 `tools/publish_map_bundle.py`，传入 **local_site恒等坐标说明、检查后的自由空间PGM和审核元数据**，以及 `--z-native-bundle`。完整命令见 [地图交付步骤](docs/Z_LOCALIZATION_MIGRATION.md#从原生z地图到rm导航地图)。

不自动把未观察区域变成自由空间，不把本次场地坐标冒充比赛官方坐标。旧RM地图缺少Z原生先验，默认导航入口会拒绝，须明确转换或重新建图。

## 5. 用地图定位、规划

先退出mapping。终端A：

```bash
bash tools/z_nav.bash state
```

终端B：

```bash
bash tools/z_nav.bash navigation /absolute/published_001/map_bundle.json local_site
```

等待 `/backend_node/localization_state` 为 `NORMAL`，且 `/localization/healthy` 为true，再通过既有 `/nav/submit_goal` 发map坐标目标。新启动不应同时运行旧冻结地图定位器或旧MapOdomManager。

例如下面请求目标 `(1.0, 0.0)`；使用时换成你检查过的地图坐标：

```bash
ros2 service call /nav/submit_goal rm_nav_interfaces/srv/SubmitGoal \
  '{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}'
```

Foxglove单纯发布 `/move_base_simple/goal` 不等同于这个监管服务，不保证会启动任务。

物理输出仍单独启用；上述命令不会打开串口。当前迁移阶段不要把新定位的“能启动”直接当作整车可放行依据。

## 架构和源码入口

```text
MID-360原始点云/IMU
  → 原始质量门 + 时间连续性门
  → ZLIO frontend（逐点时间、去畸变、6DoF里程计）
  ├→ 同包观测桥接 → 偏心补偿/nav_reference → Nav2局部避障
  └→ Z backend（原生关键帧、回环、GTSAM、先验定位/重定位）
       ├→ map→odom、优化轨迹、全历史地图、原生地图保存
       └→ 实际健康状态 → 任务监管 → 规划/控制 → 原my_serial_py
```

- `src/rm_nav_bringup/launch/z_localization_*.py`：新方案配置与启动。
- `src/rm_nav_bringup/scripts/z_localization_bridge.py`：消息、会话和真实健康接口适配。
- `src/rm_nav_bringup/scripts/sensor_reference.py`：传感器6DoF到导航轴参考点和水平速度系。
- `tools/z_nav.bash`：简化操作；`tools/bootstrap_z_localization.bash`：固定版本依赖。
- [架构契约](docs/architecture_contract.md)、[交接文档](docs/HANDOFF.md)、[阶段矩阵](docs/stage_completion_matrix.md)。旧报告保留历史证据，不能直接作为Z方案验收结果。

上游Z-Localization为 **GPL-2.0-only**，完整保留其作者、源码、LICENSE与Jazzy补丁；本工程自己的许可不覆盖上游。

## 本轮验证结果

- ASUS：18包全量构建，`colcon test-result --verbose` 汇总364项、零错误、零失败、1项可选交叉检查跳过。
- 完整327秒原始数据按采集时刻派生调度：64837条里程计、3248帧云、163次累计地图更新；末尾NORMAL、epoch0。
- 导出583关键帧、12220点地图，重新加载完整地图后的45秒先验定位通过。
- 原录包约56.64秒处有343ms接收间隔；按接收时间直接播放的故障样本仍按源年龄门拒绝。原消息、header、frame和原文件均保留，不以派生播放结果冒充实时延迟验收。

这轮未切换现有现场进程、未发送物理运动指令；场地地图检查、真实动态精度和整车导航仍需现场验收。原始报告见 [迁移记录](docs/Z_LOCALIZATION_MIGRATION.md#实际验收结果)。
