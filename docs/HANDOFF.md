# Adam_nav_2027 开发交接文档

交接日期：2026-10-09。代码基线：`f37c2172a5d9071e2027fe86d1055c241d8ca9f8`。
本文随后独立提交；上面的 SHA 指交接前的已验证代码。

## 1. 项目目标与用户要求

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
| 优化建图 | 原始关键帧持久化；真实 VGICP 相邻约束；GTSAM Pose3 图与原始云重建 | 建图回环、官方坐标对齐、动态清理、MapBundle 发布 |
| Nav2 基线 | Smac2D + MPPI Omni → smoother → collision monitor → gate，理想全向模型横移及故障停车 | 实际定位/地图/足迹与最终 TDT、Omni PID、yaw、串口适配 |
| 监管 | 本地/全局健康、运动许可、唯一任务、旧任务取消与恢复后重新规划 | 最终控制与整车运行验收 |
| 实机输入 | MID-360 官方驱动和真实静止 bag 回放通过 | 不等于动态精度或端到端整车验收 |

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

## 5. 必须保持的接口与架构约束

1. `my_serial_py` 的 `/cmd_vel` 提供 xy 平移，`/cmd_yaw_angle` 提供 yaw 目标角（度）；
   原报文不发送 `angular.z`，不能直接把 Nav2 角速度塞进目标角字段。
2. 原串口 RX 没有独立测量速度、底盘 IMU或绝对云台编码器。
   `/contact_angle` 不能代替 `/hardware/gimbal_joint_states` 的绝对角。
   `/hardware/measured_twist` 需要真实适配，EKF 估计速度不是实测停车反馈。
3. `map→odom` 由 MapOdomManager 独占；`odom→base_footprint` 由 robot_localization 独占。
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
2. **完成建图 KISS 回环。** 当前仅候选筛选原型，优化器没有回环入口/因子。
   加入有界候选搜索、真实 KISS + GICP 复核、覆盖/残差/inlier/Hessian/几何/轨迹一致性判定；
   只有接受的回环进入图。测试正确回环、误回环、重复结构和退化拒绝。
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
