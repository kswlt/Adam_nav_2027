# Z-Localization 整链迁移（2026-10-23）

用户要求以 mozihe/z-localization 替换当前定位方案。检查起点为
`453624b24230cb7aa172761478e78ce289621111`，开始前已 fetch；当时 origin/main
为 `69842dd76184c41a12ec0f446cdbcb9dffa1c31f`，实际框架开发在
`audit-p0-tf-costmap`。保留本来未提交的 `tools/smoke_local_site_navigation.py`，不加入迁移提交。

## 替换边界

| 功能 | 原实现 | 新默认实现 |
|---|---|---|
| 雷达惯性里程计、去畸变 | small_point_lio | 上游 frontend/zlio_node |
| 在线图、回环、历史地图更新 | RM 关键帧档案轮询后端 | 上游 backend/backend_node |
| 先验地图定位、全局重定位 | frozen_map_matcher、小GICP/KISS恢复 | Z 后端原生关键帧/描述子/几何匹配及状态机 |
| map→odom | MapOdomManager | 仅 Z 后端；上游 legacy TF 关闭 |
| IMU→导航轴参考点 | sensor_reference | 保留已批准的15.34cm偏心补偿；不伪造底盘朝向 |
| 规划、控制、串口 | Nav2/TDT、my_serial_py | 接口保留，未启动物理输出 |

旧组件与失败数据仍保留以供回归。`legacy_lio:=true`、`legacy_localization:=true`
仅用于历史/TEST_ONLY 回归；新的默认状态、建图与冻结地图定位入口不再选择它们。
`mid360_diagnostic.launch.py` 的默认诊断也已切换ZLIO，旧前端必须显式legacy_lio=true。
旧在线地图/关键帧菜单属于历史 release；新流程用 `tools/z_nav.bash`。

## 源码、构建与许可证

固定上游：<https://github.com/mozihe/z-localization/tree/76e126328b670adc92981d9b4900cc059a6f2343>。
上游许可 GPL-2.0-only；通过独立依赖工作区保留完整源码、LICENSE 和作者说明，
没有把上游算法复制进本工程的 Apache-2.0 文件或重新许可。分发上游及修改后的二进制时，
同时提供该版本的上游源码、许可证及本仓库补丁/配置；本仓库自己的许可声明不覆盖上游。

适配范围只有 backend CMake 启用 C（PCL/VTK MPI），以及 nanoflann 1.5
`SearchParameters`/`ResultItem`、组合入口使用外部官方驱动的launch目录。补丁见 `tools/patches/z-localization-jazzy.patch`，
支持 CRLF/LF 和幂等检查的应用器为 `tools/patch_z_jazzy.py`。
GCC/Eigen 上游头文件警告导致默认 Werror 失败，构建明确使用上游已有开关
`-DLOCALIZATION_WARNINGS_AS_ERRORS=OFF`，未改变算法门限。

```bash
# Ubuntu24.04/Jazzy；已有官方Livox驱动、GTSAM4.2、PCL、yaml-cpp、nanoflann1.5
source /opt/ros/jazzy/setup.bash
source /absolute/livox_ws/install/local_setup.bash
bash tools/bootstrap_z_localization.bash /absolute/new/z_deps_ws
cd /absolute/new/z_deps_ws
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS='-j2 -l3'
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DLOCALIZATION_WARNINGS_AS_ERRORS=OFF -DBUILD_TESTING=ON
source install/local_setup.bash
# 在RM工作区构建；其历史离线模块仍需已有KISS/small_gicp/TDT依赖前缀
cd /absolute/rm_ws
colcon build --executor sequential
source install/local_setup.bash
colcon test --executor sequential
colcon test-result --verbose
```

不要同时导入 z-localization 内置 Livox 包和外部同名驱动；bootstrap 在前者放置
COLCON_IGNORE。上游 frontend/backend/bringup 包名通用，必须在隔离 overlay 中使用，
不要把另一套同名包的 install 混进环境。

## 帧、时间与车辆几何

`z_localization_frontend.launch.py` 设置：原始 PointCloud2 的 `timestamp` 是绝对纳秒，
`point_time_absolute=true`、`point_time_scale=1e-9`；原始 IMU 的 g 乘9.81转换为SI。
只对当前已检查的官方驱动消息布局成立，更换驱动/数据集须复核字段和单位。
输入不重写时间戳、不伪造补帧。外参为 IMU←LiDAR 的工厂数值
`[-0.011,-0.02329,0.04412]m`、单位四元数；不使用上游作者的车体倾角/UWB位置。

```text
map -- Z backend --> odom -- sensor_reference --> front_mid360_imu
                        |                         └-- 工厂静态外参 --> front_mid360
                        ├-- nav_reference（导航用平移参考，虚拟yaw=0）
                        └-- sensor_yaw_reference（真实雷达水平朝向）
```

ZLIO 的 publish_tf=false；Z 后端的 output.legacy.enabled=false，禁止第二条
map→IMU TF。传感器6DoF状态保留；nav_reference不代表实际底盘姿态。
车辆 geometry YAML 保持既有用户批准的15.34cm拟合，仍标记 debug，未升级为实测 verified。
真实chassis模式保留原 resolver/EKF/投影，只替换其默认前端，仍要求真实标定及编码器。

Z 后端接收原生 FrontendFrame（点云已去畸变到该帧 IMU），不用重新时间同步。
桥接器只做同包位姿的坐标投影，生成 `/lio/deskewed_odom_cloud`；无新估计算法。
`/lio/sensor_odometry` 为 ZLIO 高频预测，须同会话且最近有真实 LiDAR 更新。
会话换代清空参考状态和旧命令；backend NORMAL、同代次、近期真实输出、输入心跳
全部满足才发布 `/localization/healthy=true`。OBS_DEGRADED也保守停止。
建图模式始终不提供导航健康许可；不存在静态 map→odom 兜底。
RM输入连续性门失效仍要求显式重启状态链，不会靠调用重置服务绕过已锁存故障。
IMU输入/输出QoS队列由5增至4000，与上游接收历史容量匹配；源时间、年龄与断流门限不变。

## 建图与地图保存

先开驱动，只保留一个雷达实例；另开终端依次执行：

```bash
bash tools/z_nav.bash mapping /absolute/maps/new_session
bash tools/z_nav.bash foxglove
bash tools/z_nav.bash status
```

停车并结束采集后，在第四个终端：

```bash
bash tools/z_nav.bash save
```

脚本检查 `/stop` 和 `/backend_node/save_map` 的真实服务返回；保存原生
`keyframes.txt`、`clouds/*.pcd`、描述子及优化后 `map.pcd`。
保存后前端停止，重开任务应先退出旧建图 launch，再重新启动。
保存目录必须每次新建；不要覆盖正在使用的先验图。Z 上游支持一致快照和原子目录事务。
保存成功不等于精度/导航自由空间通过。

## 从原生Z地图到RM导航地图

```bash
python3 tools/import_z_map.py --native-bundle /absolute/maps/new_session \
  --output /absolute/maps/new_draft --calibration-id vehicle164_yaw_axis_fit153cm_debug_v3
python3 tools/publish_map_bundle.py --draft /absolute/maps/new_draft/map_bundle.json \
  --z-native-bundle /absolute/maps/new_session --profile local_site \
  --alignment /absolute/local_alignment.yaml --output /absolute/maps/published_v1 \
  --free-space-mask /absolute/reviewed.pgm --free-space-metadata /absolute/reviewed.json
```

沿用已有人工检查的自由空间、障碍物、占据栅格/地形生成契约。
本次 Z 建图 session 的全局优化坐标本来由上游叫 map；RM draft中的 mapping_odom
只表示该局部会话坐标，数值不变，绝不声称已对齐比赛官方坐标。
local_alignment必须明确local_site、恒等变换和会话来源。
发布时保存原生Z先验图副本并绑定关键帧索引/上游版本/标定ID；导航拒绝缺少原生Z图的旧RM地图。
不能只旋转最终PCD而不旋转原生关键帧及描述子；因此目前工具拒绝Z原生图直接走official发布。
官方对齐应先用上游整包transform工具对关键帧及所有几何做一致变换，独立验证后另行接入正式发布。

## 导航与显示

退出建图后，分别开状态链和导航（不会打开串口）：

```bash
bash tools/z_nav.bash state
bash tools/z_nav.bash navigation /absolute/maps/published_v1/map_bundle.json local_site
```

全局定位会自行通过原生地图召回与几何验证；等待backend状态NORMAL、健康为true，
再经现有 `/nav/submit_goal` 下发map系目标。不要用旧MapOdomManager的恢复服务操纵Z后端。
Z请求重定位为 `/backend_node/relocalize`，仅LOST时接受；
`/localization_reset`涉及前端换代，操作前停车，状态失效会取消旧导航任务。

| Foxglove话题 | 参考系 | 用途 |
|---|---|---|
| /livox/lidar | front_mid360 | 原始当前扫描；检查输入 |
| /lio/deskewed_odom_cloud | odom | ZLIO去畸变后的当前观测，尚非历史地图 |
| /backend_node/runtime_map | map | 所有运行关键帧经后端优化重建的累计图；约2秒更新 |
| /backend_node/prior_map | map | 定位模式的已保存固定先验图 |
| /localization/lio_world_cloud | map | 后端全局定位下的当前扫描 |
| /backend_node/optimized_path | map | 优化历史轨迹 |
| /backend_node/odom_path | map | 前端里程计轨迹对照 |
| /backend_node/loop_edges | map | 接纳回环边 |
| /backend_node/localization_state | 文本 | 状态、epoch、可信性、失败原因 |
| /localization/reason | 文本 | RM健康接口的来源/故障原因 |

导入 `config/foxglove/z_localization_mapping.json`，显示坐标map。
优先只开启runtime_map与optimized_path；多路稠密云同时显示会增加网络/渲染负担。
上游runtime地图发全量快照，大场地应提高voxel或降低地图更新频率。
旧 `/visualization/live_map_preview` 为里程计累计显示，不能表示历史回环已经修正。

## 本次验证

实际编译、测试、MCAP结果和未验证事项在本文件末尾更新；历史small_point_lio通过结果
不自动计入新前端精度。不得把进程启动、单次回放或合成桥接fixture称作实车自主导航通过。

## 实际验收结果

代码提交 **`5aaf827cc5786f4714869eceefa15427b9a0d2dd`**，中文提交作者kswlt，
已推送 `kswlt/Adam_nav_2027` 的 `audit-p0-tf-costmap`，`git ls-remote` 已核对同一SHA。
README/本报告与证据在后续独立文档提交；本次没有合并main。

ASUS独立构建根目录：
`/home/asus/nav_2027_vehicle_config_164/z_migration_20261023`。
Windows本机WSL曾连接超时，本轮有效ROS构建/测试结果来自ASUS，不声称Windows原生ROS构建。
没有停止/替换原 `/home/asus/z-localization-jazzy` 的运行程序；只读确认原driver/ZLIO/backend
进程18914/18915/18916在本轮结束时仍是原进程。

### 编译和回归

```bash
source /home/asus/nav_2027_vehicle_config_164/remote_env_20261020.bash
source /home/asus/nav_2027_vehicle_config_164/z_migration_20261023/install/local_setup.bash
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib:$LD_LIBRARY_PATH"
export ROS_DOMAIN_ID=220 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS='-j2 -l3'
cd /home/asus/nav_2027_vehicle_config_164/z_migration_20261023
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DLOCALIZATION_WARNINGS_AS_ERRORS=OFF -DBUILD_TESTING=ON \
  -DGTSAM_DIR=/opt/ros/jazzy/lib/cmake/GTSAM
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
```

- 固定上游4包构建通过；加入RM后**18包全量构建3分42秒通过**。
- 修复后汇总 **364 tests / 0 errors / 0 failures / 1 skipped**。
  跳过的是原串口可选原库交叉检查，非定位测试跳过。
- 期间曾有PCL/VTK C语言、nanoflann1.5、驱动launch目录错误，已按上述补丁修复。
- 新桥接fixture初版Domain237无效、观察端QoS请求reliable不匹配sensor-data输出；
  改为合法Domain221和sensor-data订阅。属于测试配置修正，没有把生产发布强改成可靠长队列。
- 后续地图XYZI导入/嵌套输出防递归修复及IMU队列变更，重新构建并跑rm_nav_bringup回归通过。
- 新夹具验证NORMAL/LOST、同包位姿投影、错时间戳、会话失败、过期和断流；
  地图单测验证XYZI、损坏输入、原生包不匹配、越界引用及嵌套目录拒绝。
- 历史定位smoke显式 `legacy_localization:=true` 保留原测试逻辑；未改写或提交原有脏文件
  `tools/smoke_local_site_navigation.py`。它的旧GICP接口不代表新定位已被实车验收。

汇总文件：[test_result.txt](evidence/z_localization_20261023/test_result.txt)。
完整日志在ASUS `build_full.log`、`build_update.log`、`test_update.log`、
`test_imu_queue.log`、`log/`；初次失败日志保留，不覆盖为成功。

### 真实数据链与地图重载

原始数据仍为用户的
`runtime_20261022_fit153_console/bags/run_20261022_200748`，没有改写；
本机原始备份目录仍保留。测试只播放原始 `/livox/lidar`、`/livox/imu`，不播放旧位姿、
旧TF、旧健康或控制命令。独立Domain222/223、use_sim_time、/clock，无串口/电机进程。

| 数据/检查 | 实际结果 | 证据 |
|---|---|---|
| 原始前45秒（444云/8863 IMU），1倍速 | 8285里程计、416云、21次累计图更新；地图保存成功 | [mapping_45s.json](evidence/z_localization_20261023/mapping_45s.json) |
| 前45秒地图重新定位 | 8320里程计、418云、8274后端位姿，NORMAL | [prior_45s.json](evidence/z_localization_20261023/prior_45s.json) |
| 完整327秒，按采集时刻派生调度 | 64837里程计、3248云、163次累计地图更新，末尾NORMAL/epoch0 | [full_acquisition_replay.json](evidence/z_localization_20261023/full_acquisition_replay.json) |
| 完整地图导出 | **583关键帧、12220点**；原生clouds/描述子/位姿完整保存 | ASUS `replay_acquisition327s/native_map/` |
| 完整地图重载，原始前45秒定位 | 8310里程计、418云、8253后端位姿，末尾NORMAL | [full_map_prior_replay.json](evidence/z_localization_20261023/full_map_prior_replay.json) |
| TF authority | /tf只有sensor_reference和backend_node两个发布者；odom父map、IMU父odom，无旧manager | [tf_authority.txt](evidence/z_localization_20261023/tf_authority.txt) |
| 原生PCD导入RM draft | 正确处理PCL XYZ+intensity，转换RM XYZ副本；原生数据不动，583帧不截断为500 | ASUS `z_draft_full/map_bundle.json` |

完整回放中：IMU原点最大估计变化0.361665m，轴参考XY最大估计变化0.026771m，
IMU Z估计跨度0.015998m。雷达偏心旋转时IMU位置本来会移动，这两个变化量不能混称底盘漂移。
数据没有外部位置/姿态真值，**不计算ATE/RPE，不把这些数值称作实测定位精度**。
同一数据建图后再定位是链路一致性检查，不是独立泛化精度评测。

全327秒回放采样：ZLIO进程峰值RSS 65220KiB（约63.7MiB），
Z backend峰值286764KiB（约280MiB）；包含rosbag2预读器和全部测试进程的树峰值约1.27GiB。
这是本段数据与配置下的结果，不是长期/大场地内存上限；图保留历史、地图全量快照都有增长成本。

### 接收时序失败样本与派生播放的边界

原记录在约56.639秒出现接收间隔0.342621秒；IMU header最大采集间隔只有0.014025秒，
接收时间与header最大差约0.336969秒。按记录接收时间播放，会把录制端排队复现成传感器输入突发。
旧5深度IMU队列仅覆盖约25ms，曾导致前端看到大于100ms的采样间隔并重置。
增至4000后不丢弃这些排队样本，但其中0.335秒旧样本会被原0.25秒源年龄门正确拒绝；
**没有为通过回放放宽生产年龄、断流、漂移或匹配门限**。

保留的45～90秒接收顺序故障回放结果为false：
[receive_burst_expected_rejection.json](evidence/z_localization_20261023/receive_burst_expected_rejection.json)。
最初全长观察脚本只检查“曾经NORMAL+有计数”，错误地将最终LOST计为pipeline_pass；
该初版报告在ASUS `replay_full327s/report.json` 原样保留，**不计入验收通过**。
现脚本增加必须结束NORMAL、LIO覆盖数据末尾的条件，拒绝只有前段输出的“通过”。

为单独检查算法，`prepare_z_replay.py` 创建**新的派生播放文件**：
IMU按原header、点云按原逐点时间的扫描完成时刻调度；只改变新文件的播放调度时间，
所有原始CDR字节、header、frame和原文件不变。原接收时间完整记录在
`source_receive_times.csv`。固定1秒排序缓冲，实测最多214条缓存，非无限载入整包。
新文件3270云/65384 IMU，处理报告见 [scheduling.json](evidence/z_localization_20261023/scheduling.json)。

```bash
python3 tools/prepare_z_replay.py --bag ORIGINAL_RAW_BAG --output NEW_DERIVED_BAG
ROS_DOMAIN_ID=222 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
python3 tools/smoke_z_localization_bag.py --bag NEW_DERIVED_BAG \
  --output NEW_REPORT_DIR --rate 1 --max-wall-seconds 600
```

派生调度只用于离线诊断；不能用它宣称原接收顺序无故障、真实传输延迟通过或实车已可自动行驶。

### 未验证项目与交付边界

- 新方案直接接真实MID360的持续采集、断电/网络恢复与实际端到端延迟：NOT TESTED。
- 独立场地的错误重定位/重复结构、长距离运动精度：NOT VERIFIED；本次仅既有数据及上游回归。
- 正式导航可行驶区域、人审自由空间、实际占据栅格/地形通行性：NOT VERIFIED。
  当前真实地图只生成draft，不自动发布navigation_ready，也不承诺官方场地对齐。
- 新Z定位链与物理运动输出的整车闭环：NOT TESTED；my_serial_py协议未改，未启动真实发送。
- 不自动切换原现场实例；新CLI默认Domain98，旧Z实例Domain73保持运行，旧RM Domain97不接管。
- 新风险：上游针对Humble，Jazzy按固定版本及三处适配验证；通用包名overlay碰撞、
  全历史图与全量地图网络负载、厂商驱动格式变化仍需现场管理。Bootstrap固定commit、
  环境脚本核对overlay；新运行不应同时启动旧前端、旧manager或另一套同名Z包。

下一位开发者先看根README，使用新CLI与独立目录；先完成真实输入/现场地图检查，
再做新定位的实车导航验收，不要把上述离线结果当作物理放行。
