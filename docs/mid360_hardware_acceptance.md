# MID-360 实机接入记录（2026-10-09）

## 60 秒静止验收（2026-10-10）

使用单实例传感器 supervisor、正确 `.50/.3` 网络配置和外部 Livox 驱动完成 60 秒静止采集。
原始输入稳定段点云 9.86 Hz、IMU 200.02 Hz、584 帧点云、11911 条 IMU，无字段或时间错误。
固定 Point-LIO 回放输出 573 条里程计、573 帧去畸变云、58.00 s；Foxglove 可视化回归输出 573 条轨迹和
117 条实时累积地图消息，预览最多 141128 点。

静止结果：最大位置变化 0.0116 m、最终相对位移 0.0053 m、最大姿态变化 0.00313 rad；
最大相邻速度 0.08896 m/s，P99 相邻速度 0.07280 m/s。静止诊断通过；最大速度使用 P99 作为主门限，
同时保留最大瞬时值，避免 rosbag 播放调度尖峰误判真实运动。

证据：[原始静止输入](evidence/mid360_static_20261010_live.json)、
[LIO/Foxglove 回放](evidence/mid360_static_20261010_lio.json)。
该结果仍不是动态运动精度、实测外参或整车导航验收。

## 动态采集与 Foxglove 回放（2026-10-09）

使用 ROS Domain 108 采集 40 s 原始数据，随后在 Domain 109 使用固定 Point-LIO overlay 回放。
采集和回放均未启动串口、底盘或导航控制。

- 原始稳定段：点云 9.69 Hz、IMU 200 Hz、每帧有效点 12765–13461，三维协方差正常。
- LIO 回放：347 条里程计、347 帧去畸变云、35.60 s 输出，TF 发布数为 0。
- 回放链路无异常；相对首个 LIO 估计最大位置变化 8.7 mm、姿态变化 0.00110 rad。
- 该次采集没有形成可量化的大幅运动轨迹，不能作为动态精度验收；它证明了长时间输入、质量门前置检查、
  Foxglove 可视化输入和 LIO 回放链可运行。需要明确移动距离/角度的下一次采集，才能计算运动期间的连续性和漂移。

证据：[实机输入](evidence/mid360_dynamic_live_20261009.json)、
[几何检查](evidence/mid360_dynamic_geometry_20261009.json)、
[LIO 回放](evidence/mid360_dynamic_lio_20261009.json)。
原始 MCAP SHA256：`cae8e483b2fdd5a21e6b6cef0ba85320797e621f69da8e040a692fb82aeb4857`。

## 最新结果：移除遮挡后的静止对照通过

用户移动雷达、移除大部分遮挡，并明确确认采集期间保持静止。
重新采集 25 s，再用同一固定版本 Point-LIO 与相同算法参数回放。
本次增加姿态变化检查并保存 `odometry_trace.json`，没有放宽门限或修改滤波器。

| 检查 | 本次结果 |
| --- | --- |
| 稳定段原始频率 | 点云 10.00 Hz，IMU 200.00 Hz |
| 稳定段最大点云头时间差 | 0.1035 s |
| 每帧有效点（相同 tag/距离条件） | 14121–14754，之前为 0–5 |
| 回放输出跨度 | 20.6998 s，208 条里程计 / 208 帧去畸变云 |
| 从首个估计起最大位置变化 | 0.00837 m |
| 最后相对首个估计的位置变化 | 0.00445 m |
| 最大姿态变化 | 0.001577 rad，约 0.0903° |
| 相邻估计最大速度 | 0.0653 m/s |
| TF 发布数 | 0 |
| 本次静止检查阈值 | 位移 0.1 m、相邻速度 0.2 m/s、姿态变化 0.05 rad |
| 结果 | 传输、输出和静止变化检查通过 |

证据：[实时传输](evidence/mid360_unblocked_live_20261009.json)、
[原始几何](evidence/mid360_unblocked_geometry_20261009.json)、
[真实 LIO 回放](evidence/mid360_unblocked_lio_20261009.json)。
原始数据：`/home/asus/nav_data/mid360_20261009_unblocked_a/raw_bag`（118 MiB）；
MCAP SHA256：`27b186cc4f0c6fbd76f1a2ec1c6477b7e1d4378fd3fcd673f4cba9dce9d4139b`。
完整位姿轨迹留在 `/home/asus/nav_data/mid360_20261009_unblocked_lio_b/odometry_trace.json`。

```bash
source /home/asus/nav_deps/install_lio/setup.bash
ROS_DOMAIN_ID=105 python3 tools/smoke_mid360_bag.py \
  --bag /home/asus/nav_data/mid360_20261009_unblocked_a/raw_bag \
  --output /home/asus/nav_data/mid360_unblocked_retest \
  --max-speed 0.2 --max-displacement 0.1 --max-rotation 0.05
```

这些是静止条件下相对首个估计的变化，**不是带真值的定位精度**。
移除遮挡后有效点恢复、原先的大位移不再复现，支持有效几何不足是之前异常的重要因素，
但不是对全部潜在故障的排除。本次仍使用上游示例内部外参，尚未验收动态运动、
云台/底盘标定、多雷达、重定位回环或整车导航。以下保留此前失败样本作为对照。

## 网络修复

## 原始质量门与 LIO 联锁

启用 `local_state.launch.py enable_mid360_guard:=true` 后，
`rm_nav_sensors/mid360_cloud_guard` 对 `/livox/lidar` 做保守检查：必须是
`front_mid360`、单调/绝对源时间、合法 Livox 字段、至少 500 个有效回波、有效比例至少 10%，
且三维协方差最小特征值和特征值比满足阈值。通过的原始消息只做转发，不修改字段和时间。
resolver 的 `require_raw_cloud_health` 同步要求新鲜质量心跳，质量拒绝/断流时不接受新底盘位姿。

真实回归：遮挡 bag 转发 0 帧，无遮挡 bag 转发 217 帧；两种 bag 均验证断流后没有缓存重放、
健康状态保持 false。质量门单测纳入全量 37 项测试。详细报告在 `docs/evidence/`。

ASUS 的 Wi-Fi 为 `192.168.1.145/23`，有线 `enp86s0` 接 MID-360。
被动抓包发现设备 `192.168.1.3` 正在 ARP 查询主机 `192.168.1.50`。
有线配置为 `.50/24` 后，重叠网段使 SSH 回包错误地走向雷达网口。
已将有线改为 `.50/32`，只添加雷达 `.3/32` 的直连路由，并禁止有线默认路由。
Wi-Fi 地址及默认路由保留。验证雷达 ping 3/3 成功，IPv4 SSH 恢复。

本机对应 NetworkManager 配置（其他机器先核对接口、设备 IP 和连接 UUID）：

```bash
sudo nmcli connection modify uuid 117cff0b-31b9-3377-ba9e-a6204d7d1c10 \
  ipv4.addresses 192.168.1.50/32 ipv4.never-default yes ipv4.routes 192.168.1.3/32
sudo nmcli device reapply enp86s0
ip route get 192.168.1.3
ip route get 192.168.1.134
```

预期前者走 `enp86s0`，后者走 `wlo1`；此配置已持久化。

## 固定版本驱动与原始数据

SDK 和驱动的精确提交见 `dependencies.lock.yaml`。独立 overlay，SDK 安装到用户目录，
不运行会删除工作区 build/install 的上游 `build.sh`。驱动编译成功，有上游 PCL/C++ 警告。
主机与设备配置见 `config/hardware/mid360_asus.json`；SDK 点云外参保持零，未混入车体外参。

```bash
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_livox.sh
source /home/asus/nav_deps/livox_ws/install/setup.bash
ROS_DOMAIN_ID=104 python3 tools/probe_mid360.py \
  --config config/hardware/mid360_asus.json \
  --output /home/asus/nav_data/mid360_new_capture --duration 20
```

输出目录必须不存在；测试结束会关闭自己启动的驱动和 recorder。
不启动串口、底盘控制或导航。原始 rosbag 留在 ASUS 数据目录，不提交大型 MCAP。

第二次实测结果见 [原始报告](evidence/mid360_live_20261009.json)：

- 稳定段点云 **9.74 Hz**、IMU **200.00 Hz**；逐帧约 1.99–2.02 万点。
- 稳定段主机与点云头时间差最大 **0.103 s**；启动阶段存在约 2.8 s 积压，不能当作稳定延迟。
- 点云 `point_step=26`，`timestamp` 在 offset 18，为 FLOAT64 **绝对纳秒**。
  上游变量虽然叫 offset_time，实测值并非扫描起点相对时间；无需再加 header 时间。
- 实测加速度模长均值 **0.9973**：驱动原始单位是 g，直接给该 Point-LIO 时 `acc_norm=1.0`。
  不应把这个原始 IMU 当成已经转换为 m/s² 的通用 ROS IMU 输入其他融合节点。
- 点云 frame 为 `front_mid360`；原始 IMU frame 仍为驱动写定的 `livox_frame`，
  生产 Sensor Hub 仍需显式 IMU frame/单位适配；此回放仅按上游算法读取数值。
- rosbag `/home/asus/nav_data/mid360_20261009_b/raw_bag`：178 帧点云、3372 条 IMU。
  MCAP SHA256：`c358a57d294a6d410ea755b1464ccbbf325a81d47233c5de1950a2804acaac26`。

**上述通过的是传输/字段/时间检查，不是有效几何验收。**
随后对归档逐点检查发现，按当前 LIO 条件（tag 低 6 位为零、距离 0.5–30 m）
每帧只有 **0–5 个有效点**，绝大多数距离为零。
用户已确认雷达附近有遮挡；数据与遮挡相符，但没有通过无遮挡对照采集排除其他因素。
见 [原始点诊断](evidence/mid360_geometry_20261009.json)。

```bash
python3 tools/inspect_mid360_bag.py \
  --bag /home/asus/nav_data/mid360_20261009_b/raw_bag \
  --output /home/asus/nav_data/mid360_geometry_new.json
```

## 真实 rosbag 的 LIO 诊断

```bash
source /home/asus/nav_deps/install_lio/setup.bash
ROS_DOMAIN_ID=105 python3 tools/smoke_mid360_bag.py \
  --bag /home/asus/nav_data/mid360_20261009_b/raw_bag \
  --output /home/asus/nav_data/mid360_new_lio
```

测试使用原始数据、仿真时钟和固定的真实 Point-LIO，禁止发布 TF。
配置保留上游 MID-360 示例内部 IMU/LiDAR 外参，**不是实测机器人 CalibrationBundle**。
不启动 `local_state`、编码器替身或运动输出。

真实回放已产出 143 条里程计和 137 帧去畸变云，TF 发布数为零；
但最大估计位移 **71.57 m**，相邻估计最大速度 **13.55 m/s**，超过诊断范围
（10 m / 5 m/s），**整体判为失败**。见 [回放报告](evidence/mid360_lio_20261009.json)。
这些数值是异常估计，不是机器人的实际运动；没有真值，不报告精度。
第一次测试遗漏输出话题重映射导致观察者无输出，已修正后重新回放。
需要无遮挡环境重新采集，先验收有效点及几何，再进行 LIO 稳定性和导航集成。

该测试仅验证真实输入能否产生有限、正确 frame/时间的传感器里程计与去畸变点云；
没有轨迹真值，不能说明定位精度，也不替代 P0–P10 的整套导航和公开 rosbag 系统验收。
实测车体/云台外参、绝对编码器及下位机反馈仍待接入。
