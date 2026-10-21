# Adam_nav_2027 源码审计报告

2026-10-21 ASUS部署：基线3f3bb43，检查clean status并fetch后，在新release克隆/快进，保留旧工作区和配置。ASUS14包构建、56 tests零失败（1 skipped）；真实状态、只读串口和Windows实际WebSocket接收通过，物理输出关闭。新增会话管理、只读检查与限时MCAP入口，不改导航/串口算法。实测、探针API修复、原地转动关键帧限制及未验证项见 [ASUS部署记录](ASUS_SENSOR_REFERENCE_20261021.md)。

2026-10-21 电控兼容阶段：基线 b3e4be9，clean status + SSH fetch确认无新提交覆盖；复用原my_serial_py，协议编码未改。新增原缓存yaw策略的显式入口和私有速度适配，保留严格默认、超时归零和串口归属检查。WSL全14包构建、56 tests零失败（1 skipped）；原发送函数/libscrc黄金报文与新端到端PTY、旧严格模式回归通过。首次CRLF启动失败证据、修改/风险/未验收项见 [LEGACY_ADAM_REPLACEMENT](LEGACY_ADAM_REPLACEMENT.md)。本轮没有ASUS操作或物理串口输出，main不合并。

2026-10-21 传感器参考模式：本轮基线 `d6fb61b88432d7342bdb4e904735504f84a765c8`，开始前已检查 clean status 并通过 SSH fetch 确认 feature/main；未覆盖现有修改。按照用户最新要求仅本机 WSL 编译测试，ASUS未部署。新增雷达参考状态、平移影子坐标转换、独立 Nav2/TDT profile 和保守圆形包络；原协议、完整底盘模式、生产 map→odom authority 保持不变。详细问题、修复范围、测试、已知风险和未验收项见 [SENSOR_REFERENCE_MODE](SENSOR_REFERENCE_MODE.md)。

2026-10-20 在线建图补齐：基线 feature `541413d`，SSH fetch 后确认未有更新覆盖；独立实现后台调度、
复用 KISS/GICP/LoopValidator/GTSAM，历史地图事务替换，原始 CDR 读取器与离线链共用。
全 14 包在 ASUS 构建，50 tests 零失败（1 skipped）；在线/原离线闭环 ROS 回归通过，真实 session 已接入。
实测指标、修改范围、首次编译失败及修复、资源限制、新风险和未验证项详见 [在线建图验收](ONLINE_MAPPING.md)。
没有修改原车工作区、原协议、LIO 前端或生产 map→odom，没有启用物理控制；main 未合并。

2026-10-20 现场接续：开始前检查本机和 ASUS `git status` 并 fetch GitHub，feature 最新为 `45b973f`，
main 仍为 `7e2afa1`。保留两端未提交修改，在 ASUS 新建独立 release。
最新完整审计/结果见 [现场启动报告](ASUS_LIVE_20261020.md)、[导航集成](NAVIGATION_INTEGRATION_ACCEPTANCE.md)、
[建图后端](MAPPING_BACKEND_REVIEW_20261013.md)。下文为较早 P0 阶段的历史基线。

本轮 P0 环境故障是 ROS 消息预编译库与 Fast-CDR 不匹配，实际复现 controller_manager 退出；
同版本原始接口在私有 overlay 重建后 manager/标准控制器回归通过，系统包未升级。
另修复测试夹具 DDS 端点就绪竞态，保留失败证据，未放宽生产 watchdog。
ASUS 最终 14 包构建与全量测试：49 tests、0 errors、0 failures、1 skipped。
真实雷达/LIO/TF/EKF/子图/Foxglove/关键帧录制实际启动，传感器/状态健康检查通过。
真实地图、实测 footprint、yaw/制动反馈与跨主机时钟仍有未验收项目；没有接通电机输出。
各功能使用独立中文提交推送 feature，最新提交及远端 SHA 以 Git 历史和现场报告为准，main 不合并。

审计基线：远端 `main` 最新提交 `7e2afa1`（开始前已执行 `git fetch origin`）。
审计分支：`audit-p0-tf-costmap`。本轮范围只覆盖源码审计和 P0 TF/Costmap 前置问题，未进入建图 P1 修复。

## P0-1 Costmap smoke 启动完整性

**文件：** `src/rm_nav_bringup/launch/costmap_mid360_smoke.launch.py`。

当前 launch 将 `enable_lio` 和 `enable_mid360_guard` 固定为 `false`，但仍启动 resolver、EKF、
Costmap 和 PointCloud2→LaserScan 适配器。它依赖外部运行中的 `/lio/sensor_odometry`、
`/sensors/front_mid360/guarded_points` 或 `/scan`、`/state/lio_pose`、动态 `odom→base_footprint`
TF 以及测试地图。干净 ROS Domain 中这些输入不存在时，Costmap 会在 transform 或 scan 前置条件处失败。

该 launch 还发布 `smoke_odom_to_map` 静态 `map→odom`，虽然文件注释称为测试用途，但如果误在生产
Domain 启动会违反 MapOdomManager 唯一发布权。它属于 **P0 高风险**，可复现，修复范围是把启动器
明确拆成 `TEST_ONLY` 和需要外部真实输入的 `LEGACY_DEBUG`，默认拒绝生产启动；增加输入/TF/地图
缺失的可见失败状态，禁止虚拟健康和生产静态 `map→odom`。

**兼容影响：** 不改变 Nav2 参数或 `my_serial_py`；只收紧测试启动边界。**测试：** 独立
`ROS_DOMAIN_ID` 下检查节点、话题 publisher、TF authority，分别断开 scan、动态 odom TF 和地图。

## P0-2 TF authority 与 frame 语义

**文件：** `local_state.launch.py`、`gimbal_tf.py`、`legacy_static_tf.launch.py`、
`chassis_state_bridge_node.cpp`。

目标链应为 `map→odom→base_footprint→chassis→big_gimbal_yaw→front_mid360`，其中
`odom→base_footprint` 只有 EKF 发布。当前 production `gimbal_tf.py` 发布静态
`base_footprint→chassis`、`chassis→base_link` 别名、`yaw→imu`、`imu→lidar`，并动态发布
`chassis→yaw`；legacy 启动器另行直接发布 `chassis→front_mid360` 和 `chassis→front_mid360_imu`。
两套模式被重复启动时会产生重复 authority。审计期间实际观察到 `/tf_static` publisher count 为 4，
并有多组同名 `legacy_*` 节点；这是 **P0 高风险**，可复现。

`robot_localization` 当前 `two_d_mode=false`、`base_link_frame=base_footprint`，保留完整 6DoF
状态本身并非错误，但必须明确 Nav2 使用 `base_link` 还是 `base_footprint`，并由唯一状态桥做参考点
变换，不能让静态别名与 EKF 语义冲突。**修复范围：** 明确 TEST_ONLY/LEGACY_DEBUG/PRODUCTION
模式、唯一 TF 发布 authority 和启动互斥；不以 `two_d_mode=true` 丢弃车体 6DoF。

## P0-3 真实 LIO 协方差导致动态 TF 缺失

**文件：** `src/rm_nav_localization/src/sensor_pose_resolver_node.cpp`。

resolver 对 36 个协方差元素执行有限、对称、正定检查。ASUS 当前真实 `/lio/sensor_odometry`
回放可复现 `state/lio_reason: LIO covariance is not finite/symmetric`，随后
`odom→base_footprint` 不发布，Foxglove 中动态 TF 缺失。该检查拒绝无效数据是正确安全行为；根因应
追溯 Point-LIO 输出协方差的数值格式/对称化，而不是放宽阈值制造通过结果。风险 **P0 高**。

**修复范围：** 先记录原始协方差并定位上游输出；只允许有证据的数值修复（例如对称化并验证正定），
不能把零协方差当作确定性。**测试：** 合成非对称、NaN、零矩阵、正定矩阵，以及真实静止/移动 MCAP。

## P0-4 Foxglove 延迟与重复输入

**文件：** `foxglove_visualization.launch.py` 及运行实例。

运行日志实际显示 Foxglove 同时订阅 `/livox/lidar` 与 `/sensors/front_mid360/guarded_points`，并曾
存在多组旧可视化/TF 实例。原始点云和 guarded 点云都是高带宽 PointCloud2，同时发送会造成数秒排队。
这是 **P0 中风险**，可复现；低延迟参数已在本分支基线设置为 4 MiB 缓冲、50,000 体素、1 Hz，
但面板仍需只打开一个高带宽点云话题。

## P1 仅审计、不修复

- `keyframe_recorder_node.cpp` 默认 `max_keyframes=5000`，而 `offline_graph_optimizer.cpp` 和
  `pose_graph.cpp` 强制 `2..500`/`<=500`；超过 500 会拒绝优化，不是静默截断。风险 P1 高，需统一
  容量契约或分段 Submap，并测 RSS/CPU/磁盘。
- `overlap_fraction()` 在采样源点和目标点后仍使用双层距离遍历，最坏约 2000×5000；风险 P1 中高，
  应复用现有 small_gicp/PCL 搜索并做确定性结果对比。
- `loop_validator.hpp` 用 `min_condition` 同时作为 Hessian condition 与 `geometry_ratio` 阈值；风险
  P1 中，应拆分配置并增加 GTSAM 优化残差、位姿修正分布、局部形变和错误回环对照。

## 未验证项

本审计尚未修改生产 TF、未接通物理底盘、未把 legacy 外参改为 verified，未执行完整 `colcon test`。
P0 修复完成后必须在独立 Domain 运行构建、测试、TF authority、Costmap 障碍层和传感器断流回归，
再等待批准进入 P1。

## P0 修复进度

已在 feature branch 增加 Costmap smoke profile：

- `TEST_ONLY`：显式测试静态 `map→odom` 和 `odom→base_footprint`，只允许接口/Costmap 回归。
- `LEGACY_DEBUG`：依赖外部 LIO/状态输入，允许原版静态外参联调，不发布测试 `map→odom`。
- `PRODUCTION`：启动时直接拒绝，要求正式地图、MapOdomManager、实测标定和真实状态链。

ASUS 验证结果：`PRODUCTION` 拒绝通过；独立 Domain 的 `TEST_ONLY` 成功读取 20×20 测试地图，
map_server、global/local Costmap 激活，LaserScan 适配器建立订阅。使用命令行注入的零时间戳
LaserScan 未能证明障碍代价变化，因此障碍层实际 marking 仍为 **NOT VERIFIED**；下一步需要
带当前时间戳的测试发布器或真实 guarded 点云回放。

最新 ASUS 回归：重新 source Livox 和 Point-LIO overlay、恢复 `/tmp/mid360_asus.json` 后，
真实 Point-LIO 输出被 resolver 接受，`/state/lio_reason` 为 `time-aligned SE3 body pose accepted`，
`odom→base_link` 可查询。早期启动时的协方差拒绝属于启动瞬态样本，本轮未放宽校验；仍需在多次
重启和 MCAP 回放中确认不会持续复现。

构建与测试：`colcon build --symlink-install --parallel-workers 2` 成功；
`colcon test` / `colcon test-result --verbose` 为 38 tests、0 errors、0 failures、1 skipped。
# 2026-10-21 真实串口启用与默认README同步

基线883f618。按用户明确要求，新增会话physical组件，停止本工程candidate后打开真实ttyUSB0；复用现有legacy_adam_transport及my_serial_py，协议与控制算法未修改。实测2.013秒100次54字节成功write，xy=0、缓存yaw=0°；不视为固件执行回执。保留此前56项构建测试结果，本次未重跑全包构建；新增运维脚本执行语法/会话测试，现场物理组件启动成功。检查器补充私有serial_healthy发布者检查，避免将全局cmd_vel无人发布误报为未接通串口。未确认非零速度方向、遥控优先级、停车反馈；未启动自动导航。证据见evidence/asus_serial_tx_20261021.json。Git SHA与远端结果由本次提交及最终报告记录。
