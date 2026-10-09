# MID-360 实机接入记录（2026-10-09）

## 网络修复

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
