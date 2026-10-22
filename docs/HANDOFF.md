# Adam_nav_2027 开发交接文档

**中文终端菜单：** 新增 `tools/deployment/20261021/nav_console.py`，复用受标记会话管理器；建图、录包、新会话在线优化、draft导出、地图选择、影子导航、日志与消息集中操作。真实串口是独立选项。管理器新增navigation组件和无缓冲Python日志。3项无设备CLI回归及语法检查通过；未做ASUS实机整流程。详见 [操作与Foxglove配置](NAV_CONSOLE.md)。

**累计预览更新：** 修复live_map_preview的LRU删旧区域，改为容量受控的全局体素粗化；保留旧区域覆盖、真实源时间与断流stale，晚连接者收到快照。WSL重建/回归通过，汇总64 tests零失败/1跳过；纯核心容量测试5万/20万点峰值约61/121 MiB，不含ROS/LIO/浏览器。用法见 [累计点云指南](ACCUMULATED_MAP_VIEW.md)，尚未部署ASUS。

**安装语义最新更正：** 用户明确16 cm是yaw轴→雷达偏心，随后批准使用录包拟合值。车辆入口 `vehicle_164_sensor_reference_state.launch.py` 当前使用前1.53cm、左15.27cm（半径15.34cm），参考点定义在yaw轴与LiDAR同高处；`reference_fit_debug_approved=true`、实测verified保持false。C→G未知，完整底盘标定仍draft。新ID为 `vehicle164_yaw_axis_fit153cm_debug_v3`，旧包络不再视为已验证。详见 [安装更正](VEHICLE164_GEOMETRY_CORRECTION_20261022.md)，未部署ASUS。

**2026-10-22修复进展：** 新增 [旋转参考链修复报告](SENSOR_REFERENCE_FIX_20261022.md)。两帧真实LIO的有界yaw外推修复50Hz转换滞后，拒绝乱序速度；轴心模式要求实测几何和新标定ID。WSL新目录全14包构建，61 tests零失败/1跳过；实际MCAP状态链1170/1180帧匹配、XY差0；原Adam串口PTY通过。未部署ASUS、未改实车外参、未发实车命令。上一条“本机没有ROS环境”仅指Windows终端，现已使用本机WSL Jazzy完成软件回归。

**2026-10-22最新：** 用户明确ASUS当前不在，改为本机重读完整MCAP/UART。新增 [旋转原始数据复核](GIMBAL_OFFLINE_ANALYSIS_20261022.md) 与可复现工具：两段旋转的LiDAR候选偏心15.34/15.27cm，未变更生产外参；第三组运动不能由该偏心单独解释。角度时间交集已纠正，第一份RX未覆盖后续明显旋转。新stamped角度消息的ASUS构建仍NOT TESTED。后文设备运行状态均为历史快照。

**当前最新：请先看 [1米往返与物理速度实测](VEHICLE_MOTION_COMMISSION_20261021.md)。** 已完成1m/s×1s真实写串口，用户观察底盘移动后停下；58项测试0失败/1跳过，ASUS14包构建通过。driver启用独立异步发布后60秒含录包检查未再次锁止，长时稳定性仍未验证。短往返60帧已优化为draft，未发布导航地图。设备随后反复断电，暂停实车采集与运动，恢复后需重查进程；物理工具已退出、串口已关闭，未下发Nav2目标。下方旧快照只作历史证据。

**最新完整交接（2026-10-21）：请先读 [LIO修复与实车部署完整交接](HANDOFF_20261021_LIO_REPAIR.md)。** 已修输入断流继续预测及异常位姿未拦截；ASUS14包/57 tests零失败、1跳过；静止60秒LIO三维最大变化9.2mm。当前driver/state/Foxglove运行，capture/online/physical/Nav2均未运行；新contact_angle收到115.624～115.646，语义尚待确认。下方记录均保留为历史，不得把旧运行状态覆盖到当前。

2026-10-21地图交付实际检查：用户选择本地场地坐标。原16帧会话第0→1帧估计跳4762.076m，在线fault，离线优化返回1拒绝；已停本项目capture，原始档案保留。导出故障前2806点/1帧检查包、占据PGM/YAML及最低观测z地形，navigation_ready=false；不是完整地图。详见 [地图导出报告](MAP_EXPORT_20261021.md)。正式发布受LIO异常阻塞，本地场地契约也尚需与官方契约分开实现。

2026-10-21电控ZIP源码审计：新增 [yaw链路说明](ELECTRICAL_YAW_AUDIT_20261021.md)。contact_angle在云台板写死0；底盘板内部大yaw编码器用于xy变换，因此雷达参考方案不需要角度回传。上位机yaw_degrees与电控yaw_speed含义冲突：后者在Navigation、无自瞄跟踪、running_state=5时作为deg/s；默认导航扫描80°/s。发现Navigation_online递减但消费无断流清零门控、底盘Navigation W_set=0。只做源码审计，未改动/烧录电控；待确认ZIP与实车固件一致，不能按旧字段名直接发非零yaw。

2026-10-21最新：用户明确要求真实串口发送，已停candidate并启动physical（只管理本项目进程组）。ttyUSB0实测2.013秒成功写入100个54字节报文；xy=0，原版缓存yaw=0°。尚未启动自动导航，非零运动与电控执行未验收。README新增启动/停止/查询命令；下方TX=0为此前只读快照。

2026-10-21 ASUS已完成部署：独立新release `20261021_sensor_reference`、新运行目录 `runtime_20261021_sensor_reference`；ASUS14包构建、56 tests零失败（1 skipped）。真实雷达/LIO/参考TF/子图、Foxglove、新关键帧及在线地图已运行。电控只读1664帧、CRC0、TX0，没有物理发送或自动导航。操作入口已更新README；请先读 [ASUS现场记录](ASUS_SENSOR_REFERENCE_20261021.md)。下方“ASUS未部署”属于此前历史状态。

2026-10-21 电控兼容补齐：新增 `legacy_adam_transport.launch.py`，默认不打开串口，使用独立候选速度避免原区域节点 `/cmd_vel` 混入。`my_serial_py` 保留54/26字节原协议，仅增加显式 legacy_cached yaw策略；默认 require_fresh 不变。不发布替代业务yaw目标。原发送函数/libscrc黄金报文逐字节通过，WSL14包构建、56 tests零失败（1 skipped），整链路PTY约50.8Hz，旧严格串口回归也通过。ASUS未部署、未动车、原代码未修改；先读 [原Adam兼容说明](LEGACY_ADAM_REPLACEMENT.md)。

2026-10-21：用户已批准传感器参考导航方案，并要求暂不部署 ASUS。本次接入 `sensor_reference_state.launch.py` / `sensor_reference_navigation.launch.py`，默认不依赖 contact_angle，不发布假 chassis TF，输出只到影子速度。主 README 已重写为按操作顺序的说明；先读 [雷达参考模式](SENSOR_REFERENCE_MODE.md)。本机14包构建、53 tests / 0 errors / 0 failures / 1 skipped通过；新参考模式、原TDT/MPPI导航和nav_reference在线回环回归通过。下方“尚未接入”属于此前状态，已被本条取代。

2026-10-20 原版接口审计更正：原版及 ASUS 原 Docker 的 fake_vel_transform 都从 LIO 衍生 odometry 读取 yaw，
把虚拟参考速度旋转后交 my_serial_py 发送，不依赖 contact_angle；原协议说明将其标为“大小yaw差角”。
不能把该字段为0判成所有导航方案都不能运行。完整真实 chassis 方案与传感器参考兼容模式应区别处理；
后者尚未在新框架接入，需复用原电控速度坐标约定及覆盖偏心车体的避障包络，不能直接绕过现有动态健康门。
源码位置、已确认事实和电控侧未知项见 [原版速度坐标审计](ADAM_CONTROL_FRAME_AUDIT_20261020.md)。

2026-10-20 最新硬件澄清：用户确认雷达随 yaw 转，外轮廓 50×50 cm、轴左偏 16 cm、LiDAR 离地 25 cm。
已准备 geometry YAML 与动态标定 draft；真实角度源/方向/零点/时间仍待验证，禁止把旧 fixed profile 当作该安装的实车定位验收。
两轮串口只读均成功（12 秒3997帧、45秒14985帧，CRC错误0、TX=0），contact_angle 均为0。
为避免新转动被旧固定安装配置错误归档，已暂停本工程旧关键帧录制与在线优化，原始雷达/LIO/Foxglove 保留。
细节和动作时间验证状态见 [安装及电控记录](VEHICLE_DIMENSIONS_PENDING_20261020.md)；下方在线“运行”描述暂停前状态。

2026-10-20 最新：已接入并在 ASUS 当前真实 session 启动在线回环/历史地图重建，
使用独立 `build_online/install_online`；当前 29 帧、6 条通过门限回环、积压 0，真实闭环真值尚未验证。
全量 50 tests 零失败（1 skipped），在线 ROS 闭环和原离线回归通过。新 Foxglove 布局为
`mapping_online.json`、参考 mapping_odom，详情见 [在线建图指南](ONLINE_MAPPING.md)。
原 driver/state/recorder/Foxglove 保留，没有物理输出；下方“8765 未监听”只描述此前文档更新时的状态。

2026-10-20 使用文档更新：操作入口改为 [README](../README.md)，包含当前 `.164:2233` 独立部署启动顺序、
建图保存、导航前置条件与 Foxglove 话题/TF/颜色/曲线说明；[Foxglove 详细指南](foxglove_visualization.md) 补充排查步骤。
本次 SSH 检查没有发现 Livox、small_point_lio、Foxglove 进程，8765 没有监听，因此下文“已启动”描述此前验证，
不能视为当前后台服务在线。本次只修改文档，未重启节点、改参数或开启物理控制；未重复运行 colcon 构建测试。
文档校验包括本地相对链接、源码中的话题/参数/输出文件名与 git diff；提交号和推送状态以对应 Git 提交为准。

2026-10-20 最新接续：优先阅读 [ASUS 现场启动报告](ASUS_LIVE_20261020.md)、
[完整使用与架构说明](SOFTWARE_USAGE_20261013.md)、[导航集成验收](NAVIGATION_INTEGRATION_ACCEPTANCE.md)。
全 14 包在本机 WSL 和 ASUS 构建测试通过，49 tests 零失败（1 skipped）；真实 MID360/LIO/TF/EKF/
子图/Foxglove 已在独立 release 启动。TDT、正式地图软件发布契约与 ros2_control 反馈接口已接入，
真实车辆运动、官方地图和反馈验收仍未完成。Git SSH 身份为 kswlt；旧“推送阻塞”只描述历史状态。

交接日期：2026-10-10。代码基线：以远端 `main` 最新提交为准。
本文随后独立提交；上面的 SHA 指交接前的已验证代码。

## 1. 项目目标与用户要求

本轮第一阶段任务审计见 [第一阶段功能审计矩阵](phase1_function_audit.md)。审计基线已经先执行
`git fetch origin`，以 `audit-p0-tf-costmap` 最新提交为准；ASUS 上未提交的
`offline_graph_optimizer.cpp` overlap 优化被保留，不能在同步文档时覆盖。

根据 `RM_Nav_V2_技术方案_v2.5_NUC15Pro_重定位回环_GitHub同步 (1).pdf`
完成 P0–P10 导航框架，保持结构清晰，可参考、复用其他学校开源代码。
原版为 `adam深北莫开源改版代码-上场版`；上位机/下位机通信必须复用 **my_serial_py**，
不能误用 `standard_robot_pp_ros2`。

用户已要求在 ASUS 上构建、验证，完成每个独立步骤后推送 GitHub；
提交说明使用中文，author 和 committer 均使用 `kswlt`。
最终仓库是 [kswlt/Adam_nav_2027](https://github.com/kswlt/Adam_nav_2027)，
早先的 `kswlt/nav_2027` 已不是后续目标仓库。
全部框架完成后，开发者必须自行寻找兼容公开 rosbag，进行系统测试、修复问题并提交报告。

**当前 P0–P10 尚未全部完成。** 不得把包骨架、合成测试或静止回放描述为完整实车导航验收。

## 2. 工作区、主机与版本

| 项目 | 位置或状态 |
| --- | --- |
| Windows 总目录 | `C:\Users\Admin\Desktop\新导航框架` |
| Windows 工程副本 | `C:\Users\Admin\Desktop\新导航框架\rm_nav_v2`，本地无 `.git` |
| 技术方案 PDF、原版及参考学校源码 | 位于 Windows 总目录；不在远端工程内默认提供 |
| SSH | `ssh asus@192.168.1.145`；凭据由用户单独提供 |
| ASUS 系统 | Ubuntu 24.04、ROS 2 Jazzy，目标 ASUS NUC 15 Pro |
| 权威 Git 根目录 | `/home/asus/nav_2027/rm_nav_v2`，不是它的父目录 |
| ROS 安装 | `/opt/ros/jazzy` |
| 第三方依赖 | `/home/asus/nav_deps` |
| 通用依赖前缀 | `/home/asus/nav_deps/install` |
| Point-LIO overlay | `/home/asus/nav_deps/install_lio` |
| Livox 独立工作区 | `/home/asus/nav_deps/livox_ws` |
| 实机 rosbag / 报告 / 位姿轨迹 | `/home/asus/nav_data`，不放入 Git |
| Git origin | `git@github.com:kswlt/Adam_nav_2027.git` |
| Git 分支与身份 | `main`；`kswlt <kswlt@users.noreply.github.com>` |

交接时远端工作区干净；未发现本次测试遗留的 Livox、Point-LIO 或 bag 播放/录制进程。
可用磁盘约 9.5 GiB，采集/下载前重新检查 `df -h`。
该机内存约 7 GiB，常规编译限制 `MAKEFLAGS=-j2`，GTSAM bootstrap 使用单编译进程。
本地文件同步不是 Git 提交；提交和推送必须在上述远端 Git 根目录执行并核验结果。

Windows 可用 PuTTY `plink` / `pscp`。当前会话使用的 ASUS SSH 主机指纹：
`SHA256:VMfripMIt2DwA9GBQaJxq5Mt+h+ZjtshsDHgqmzJRIU`。
此前 IPv4 路由故障期间通过 IPv6 `2408:8215:51b:4250:561b:bcab:4a02:96da`
恢复连接；IPv6 地址可能变化，不能当作固定部署地址。
不要把登录密码、私钥或 token 加入公开仓库。

## 3. MID-360 网络和当前接入状态

用户已接入 MID-360，并在最新采集前移动雷达、移除大部分遮挡；明确确认采集期间保持静止。

| 接口 | 地址与用途 |
| --- | --- |
| Wi-Fi `wlo1` | `192.168.1.145/23`，SSH、外网和默认路由 |
| 有线 `enp86s0` | `192.168.1.50/32`，仅访问雷达 |
| MID-360 | `192.168.1.3`；主机目标地址 `.50` 来自实际 ARP 抓包 |
| 雷达直连路由 | `192.168.1.3/32 dev enp86s0` |
| 驱动配置 | `config/hardware/mid360_asus.json` |

曾把有线设为 `.50/24`，与 Wi-Fi 网段重叠，导致 SSH 回包走错接口。
已持久化为 `/32` 地址和单设备路由；不要恢复成会抢占 Wi-Fi 路由的 `/24`。
NetworkManager UUID 和修复命令见 [实机接入记录](mid360_hardware_acceptance.md)。

驱动使用固定版本官方 Livox SDK2 / livox_ros_driver2，在独立 overlay 构建。
SDK 安装在用户前缀。不要直接运行上游 `build.sh`：它会删除其上层 build/install。
改用本项目 `tools/bootstrap_livox.sh`，精确提交见 `dependencies.lock.yaml`。

实际接口已确认：

- `/livox/lidar` 是 PointCloud2，FLOAT32 XYZ、UINT8 tag、FLOAT64 timestamp；
  `point_step=26`，timestamp 在 offset 18，值是绝对纳秒，不能再次加 header 时间。
- 稳定段点云 10 Hz、IMU 200 Hz，点云头相对主机时间最大差约 0.1035 s；
  启动阶段存在约 2.8 s 积压，不能作为稳定延迟指标。
- 原始 `/livox/imu` 加速度单位为 g，静止模长约 0.9923；
  直接喂当前 Point-LIO 时 `acc_norm=1.0`。此前合成 SI 输入用的是 9.81，两者不能混用。
- 点云 frame 为 `front_mid360`，驱动原始 IMU frame 是 `livox_frame`。
  生产 Sensor Hub 仍需显式处理 IMU frame/单位，不能直接把该消息当作 SI 底盘 IMU。
- SDK 点云外参保持零；回放使用上游示例内部 IMU/LiDAR 外参，尚非实测机器人标定。

## 4. 已完成内容和证据边界

工程共 14 个包，最近全量构建通过；交接时读取现存测试结果为
**36 tests、0 errors、0 failures、1 skipped**，本次交接没有重新执行全套测试。
跳过项是可选 libscrc 兼容核验；历史报告记录单独加载 libscrc 1.8.1 后串口 13 项通过。

| 范围 | 当前实现和已验证内容 | 剩余边界 |
| --- | --- | --- |
| 串口 | 保留原版；26-byte RX / 54-byte TX、CRC、符号/yaw/话题、PTY 和平移超时停车 | 实车抓包、断线重连、下位机 watchdog、完整停车联调 |
| 主 LIO 与状态 | 固定上游 small_point_lio；原始 IMU 位姿/完整协方差；时间对齐 SE(3) resolver；真实 robot_localization | 实测外参、绝对编码器、轮速/底盘 IMU及动态真实回放 |
| 观测与局部定位 | 原生观测来源/原点/角色；有界子地图；真实 small_gicp GICP | 真实移动序列和多源感知 |
| 丢失恢复 | 真实 KISS 粗配准 + GICP 精化、独立候选复核、受限搜索、恢复事务及新任务放行 | 真实重复场景/丢失恢复及硬件停车反馈 |
| 优化建图 | 原始关键帧持久化；真实 VGICP 相邻约束；GTSAM Pose3 图与原始云重建；回环候选预算/双向质量门接口 | 回环边接入 GTSAM、官方坐标对齐、动态清理、MapBundle 发布 |
| Nav2 基线 | Smac2D + MPPI Omni → smoother → collision monitor → gate，理想全向模型横移及故障停车；提供默认关闭的 MID-360 点云→LaserScan 适配 | 实际定位/地图/足迹与最终 TDT、Omni PID、yaw、串口适配 |
| 监管 | 本地/全局健康、运动许可、唯一任务、旧任务取消与恢复后重新规划 | 最终控制与整车运行验收 |
| 实机输入 | MID-360 官方驱动、60 秒静止和约 1 m 移动回放、原始点云质量门和 Foxglove 轨迹链 | 外部运动真值、实测外参、动态精度或端到端整车验收仍未完成 |

MPPI Omni 是技术路线的早期可运行基线，不替代最终 TDT/Omni PID 方案。
详细状态以 [阶段矩阵](stage_completion_matrix.md) 和各模块验收报告为准。

### 最新实机对照

| 指标 | 遮挡失败样本 | 最新无遮挡、静止样本 |
| --- | --- | --- |
| LIO 条件下每帧有效点 | 0–5 | 14121–14754 |
| 输出跨度 | 约 15 s | 20.6998 s |
| 最大估计位置变化 | 71.57 m | 0.00837 m |
| 最终相对首个估计位移 | 不用于验收 | 0.00445 m |
| 最大姿态变化 | 旧报告未记录 | 0.001577 rad，约 0.0903° |
| 输出 | 143 条位姿 / 137 帧云 | 208 条位姿 / 208 帧云 |
| 判定 | 失败 | 本次静止诊断通过 |

静止门限为位移 0.1 m、相邻估计速度 0.2 m/s、姿态变化 0.05 rad；TF 发布数为零。
这些是相对首个估计的变化，不是有轨迹真值的定位精度。
遮挡消除后有效点恢复、异常不再复现；保留失败样本，不删改或伪造通过结果。
报告和哈希见 [实机接入记录](mid360_hardware_acceptance.md) 及 `docs/evidence/`。

### 原始点云质量门（最新）

`rm_nav_sensors/mid360_cloud_guard` 检查原始 Livox PointCloud2 的 frame、源时间、
字段布局、有效回波数量、有效比例和三维协方差几何；通过时逐字节保留原始 `data`、
字段、frame 和源时间发布到 `/sensors/front_mid360/guarded_points`，不重采样、不重打时间。
质量状态发布到 `/sensors/front_mid360/cloud_healthy`；`local_state.launch.py` 的
`enable_mid360_guard:=true` 会将 Point-LIO 输入切换到 guarded 话题，并要求 resolver
同时收到新鲜质量心跳。拒绝或断流会停止新的底盘位姿健康输出，不会重放缓存。

真实回归结果：遮挡样本 0 个点云被转发，原因为 `Insufficient valid returns for LIO`；
无遮挡样本 217 个点云被转发，原始 payload/时间/步长保持一致；两者断流后均保持不健康。
质量门单测和完整工程回归为 37 tests、0 errors、0 failures、1 skipped。
报告见 `docs/evidence/mid360_guard_bad_20261009.json`、
`mid360_guard_good_20261009.json`、`raw_quality_interlock_20261009.json`。

## 5. 必须保持的接口与架构约束

1. `my_serial_py` 的 `/cmd_vel` 提供 xy 平移，`/cmd_yaw_angle` 提供 yaw 目标角（度）；
   原报文不发送 `angular.z`，不能直接把 Nav2 角速度塞进目标角字段。
2. 原串口 RX 没有独立测量速度、底盘 IMU或绝对云台编码器。
   `/contact_angle` 不能代替 `/hardware/gimbal_joint_states` 的绝对角。
   `/hardware/measured_twist` 需要真实适配，EKF 估计速度不是实测停车反馈。
3. `map→odom` 由 MapOdomManager 独占；robot_localization 输出完整 `odom→chassis` 状态，
   `base_footprint_projection` 独占发布平面 `odom→base_footprint` 和动态 chassis 残差。
   LIO 只发布原始传感器位姿；不发布竞争 TF。重定位不重置 LIO/odom。
4. resolver 必须按源时刻查询云台外参，保留完整 SE(3) 和协方差；不回退到最新 TF。
   未实测的 CalibrationBundle 模板不能改成 verified 冒充实测。
5. `/lio/deskewed_odom_cloud` 已在 odom，不能再重复施加传感器外参。
   不把已去畸变 XYZ 凭空标成带逐点采集时间或射线清障证据。
6. 本地/全局健康和运动许可必须新鲜；不造恒 true 心跳、编码器或停车反馈。
   恢复事务提交后等待新规划，不能恢复旧缓存命令。
7. `/nav/cmd_vel_safe` 目前与物理串口隔离；未完成标定、yaw 和停车联调前不要直接接线放行。
8. 离线地图输出目前是 `mapping_odom`，未做官方对齐、未发布 MapBundle，
   不能直接声称它是生产冻结地图。

## 6. 关键代码入口

以下路径相对远端 Git 根目录。

| 入口 | 用途 |
| --- | --- |
| `src/my_serial_py/` | 原协议与兼容桥；原版入口保留作对照 |
| `src/rm_nav_bringup/launch/local_state.launch.py` | verified 标定 → 云台 TF → LIO → resolver → EKF → 状态桥/观测/子地图 |
| `src/rm_nav_localization/src/sensor_pose_resolver_node.cpp` | 源时刻外参、完整 SE(3)、协方差和编码器健康 |
| `src/rm_nav_localization/src/chassis_state_bridge_node.cpp` | 底盘参考点转换及健康联锁 |
| `src/rm_nav_sensors/src/lio_observation_adapter_node.cpp` | 主 LIO 原生观测来源与原点 |
| `src/rm_nav_localization/src/observation_submap_node.cpp` | 滚动 odom 子地图 |
| `src/rm_nav_bringup/launch/frozen_map_localization.launch.py` | GICP/KISS、map→odom、恢复链 |
| `src/rm_nav_bringup/launch/mapping_capture.launch.py` | 原始关键帧采集，需显式存储根和标定 ID |
| `src/rm_nav_mapping/src/offline_graph_optimizer.cpp` | 归档读取、真实 VGICP、GTSAM、重建与输出事务 |
| `src/rm_nav_mapping/src/pose_graph.cpp` | 当前仅相邻链 GTSAM Pose3 优化 |
| `src/rm_nav_mapping/src/loop_candidate_manager.cpp` | 时间/空间候选原型，尚非完整回环 |
| `src/rm_nav_registration/` | 真实 GICP/VGICP/KISS 后端和质量数据 |
| `src/rm_nav_bringup/launch/mppi_baseline.launch.py` | Nav2 基线及监管/输出门 |
| `docs/mppi_pointcloud_scan_adapter.md` | MID-360 点云到 `/scan` 的可选适配和实测 TF 前置条件 |
| `tools/patches/small_point_lio_raw_state.patch` | 集中维护上游发布接口补丁 |

## 7. 构建与重现命令

已有依赖可直接复用，不必每次重新 bootstrap。新机器按 README 的依赖顺序安装，
固定提交在 `dependencies.lock.yaml` / `dependencies.repos`，各 bootstrap 会检查上游本地修改。
遵守上游许可证、保留来源，不复制算法输出替代真实算法。

ASUS 上使用干净 shell 构建，避免先 source 本工程旧 install：

```bash
cd /home/asus/nav_2027/rm_nav_v2
source /opt/ros/jazzy/setup.bash
source /home/asus/nav_deps/install_lio/setup.bash
MAKEFLAGS=-j2 CMAKE_PREFIX_PATH=/home/asus/nav_deps/install:${CMAKE_PREFIX_PATH:-} colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

按改动范围选择回归，不需要文档修改也重跑所有长测试：

```bash
# 上游 LIO → 动态状态 → 观测/子地图 → 原始关键帧；输入为明确合成数据。
ROS_DOMAIN_ID=103 python3 tools/smoke_small_point_lio.py --with-state --with-mapping
# 真实 VGICP/GTSAM，归档输入是显式合成漂移 fixture。
python3 tools/smoke_offline_mapping.py
# Nav2 和输出门；模型反馈，不是实车制动。
ROS_DOMAIN_ID=87 python3 tools/smoke_mppi_baseline.py
ROS_DOMAIN_ID=90 python3 tools/smoke_motion_gate.py
```

真实静止 bag 重现，不打开物理串口：

```bash
source /home/asus/nav_deps/install_lio/setup.bash
ROS_DOMAIN_ID=105 python3 tools/smoke_mid360_bag.py \
  --bag /home/asus/nav_data/mid360_20261009_unblocked_a/raw_bag \
  --output /home/asus/nav_data/mid360_handoff_replay_new \
  --max-speed 0.2 --max-displacement 0.1 --max-rotation 0.05
```

重新采集及检查有效点：

```bash
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_livox.sh  # 仅需安装/重建时
source /home/asus/nav_deps/livox_ws/install/setup.bash
ROS_DOMAIN_ID=104 python3 tools/probe_mid360.py \
  --config config/hardware/mid360_asus.json \
  --output /home/asus/nav_data/mid360_handoff_capture_new --duration 25
python3 tools/inspect_mid360_bag.py \
  --bag /home/asus/nav_data/mid360_handoff_capture_new/raw_bag \
  --output /home/asus/nav_data/mid360_handoff_geometry_new.json
```

输出目录/报告文件必须不存在；换新名字，不覆盖旧证据。
probe 的 passed 只覆盖传输、字段、时间；有效几何必须另读 inspect 报告，
不能因为零点也是有限值就认为扫描可定位。所有测试使用独立 ROS_DOMAIN_ID。

优化建图须先停止采集：

```bash
source /home/asus/nav_2027/rm_nav_v2/install/setup.bash
ros2 run rm_nav_mapping offline_graph_optimizer /absolute/completed_session /absolute/new_output
```

当前只接受 2–500 帧连续编号完整 session，活动锁、坏相邻边、混合标定、partial、
已有输出等均拒绝。原始 CDR/hash 保持不变；重建使用优化位姿变换原始归档云。
输入限制与产物见 [离线建图验收](offline_mapping_acceptance.md)。

## 8. 下一步执行顺序与验收要求

以下为待办，不能当作已经完成的功能。

1. **完善真实输入适配和质量保护。** 明确原始 IMU 的 frame/单位、有效点/几何、时间异常和断流边界。
   把现有遮挡失败 bag 用作退化输入拒绝案例，确保不能仅凭持续输出位姿宣称健康。
   采集受控动态序列后检查去畸变、连续性、延迟和资源；记录实际运动条件，不用静止数据代替。
2. **完成建图 KISS 回环。** 候选预算、双向质量门、KISS 正反向配准和 GTSAM validated loop edge/Huber 接口已完成；
   `offline_graph_optimizer --enable-loops` 已能生成并提交通过验收的回环边；离线 smoke 已覆盖无候选和受控返回路径，
   后者实际接受 3 条 KISS/GICP 双向回环边，并限制每个当前关键帧最多一条回环边。仍需真实场地回环、错误回环和长期地图数据验收。
   继续用真实场地数据测试错误回环、重复结构和退化拒绝；当前只有通过所有质量门的回环进入图。
   不能把已完成的 KISS 丢失恢复宣称为建图回环。
3. **扩展图优化验收。** 当前 `pose_graph.cpp` 的最终残差门限针对可拟合的相邻链；
   含噪回环通常有非零残差，需设计合理因子噪声、鲁棒核和优化后约束检查，
   不能机械沿用近零目标要求，也不能直接删除验收约束。
4. **完成地图交付链。** 官方场地坐标对齐、动态点/地图清理、MapBundle 元数据/哈希/版本/原子发布，
   再接入冻结地图定位；保持 mapping_odom 与正式 map 坐标的区分。
5. **完成现场状态与多源感知。** 测量底盘/云台/IMU/LiDAR 外参，接绝对编码器、实测轮速/停车反馈，
   按技术方案完成多雷达、障碍/地形/窄道和代价地图输入。
6. **完成最终规划控制。** 接 TDT、时序轨迹验证、Omni PID、YawManager、ros2_control/STM32 适配；
   用实测尺寸/制动数据替换初始 footprint/停止区。保留 MPPI 基线作为回归对照。
7. **完成系统与实车验收。** 真实地图定位、丢失恢复、重复结构误配准、串口/下位机断流、任务取消、
   新规划放行、CPU/RAM/队列/磁盘和长时间运行，按证据更新 P0–P10。
8. **框架完成后自行找公开 rosbag。** 核对官方来源、许可、外参、逐点时间、格式和真值，
   需要转换则检查前后帧数/时间/几何。自行下载、测试、修复并推送报告。
   录制轨迹不会受当前控制命令改变，bag 回放不能单独证明闭环到达或制动。
   详细强制要求见 [公开 rosbag 系统测试计划](rosbag_system_test_plan.md)。

## 9. 提交和继续工作约定

Foxglove 可视化已接入官方 `ros-jazzy-foxglove-bridge` 3.6.0。
使用 `ros2 launch rm_nav_bringup foxglove_visualization.launch.py` 后，Windows Foxglove Studio
连接 `ws://192.168.1.145:8765`；可选 `map_pcd:=... map_frame:=mapping_odom` 发布二进制 XYZ PCD 到
`/visualization/map_cloud`。完整话题清单、实时建图启动顺序和 Frame/时间边界见
[Foxglove 可视化说明](foxglove_visualization.md)。Bridge 只可视化，不启动底盘或串口。

每个可审阅的独立步骤：ASUS 验证 → 更新 README/阶段矩阵/报告 → 中文 commit → push → 核验远端 SHA。
延续当前功能需求，不因用户说“继续”就丢失前面的 P0–P10、通信和系统测试要求。
Windows 修改后明确同步所改文件，避免覆盖 ASUS 上未提交工作。

```bash
cd /home/asus/nav_2027/rm_nav_v2
git status --short
git remote -v
git config user.name kswlt
git config user.email kswlt@users.noreply.github.com
git diff --check
# git add -- <本步骤明确修改的文件>
# git commit -m '本步骤的中文说明'
git push origin main
git log -1 --format='%H%n%an <%ae>%n%cn <%ce>%n%s'
git ls-remote origin refs/heads/main
```

不要再次 `git init`、重建 main 或为更换远端而丢弃历史；当前仓库已配置完成。
近期关键提交：

| SHA | 内容 |
| --- | --- |
| `f37c217` | 无遮挡静止回放、姿态漂移检查和报告 |
| `9b31feb` | MID-360 官方驱动、网络修复记录、遮挡失败对照 |
| `769f7b8` | 明确框架完成后公开 rosbag 系统测试要求 |
| `4caee6f` | 真实 VGICP 相邻约束、GTSAM 与离线原始云重建 |
| `a5bdb76` | 底盘触发的原始关键帧归档 |

接手先看本文、README 和阶段矩阵；涉及模块再读对应验收报告。
完整技术路线仍以用户指定 PDF 为依据，本文记录当前实现与接续入口，不替代技术方案。
