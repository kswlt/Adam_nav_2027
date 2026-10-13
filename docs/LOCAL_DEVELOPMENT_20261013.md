# 2026-10-13 本机代码开发与验收

后续接续记录见 [轨迹跟踪验收](TRAJECTORY_TRACKER_ACCEPTANCE.md)：已增加时序轨迹核心与 ROS 候选输出入口，
并修复 Omni 未就绪期间接收命令可能在恢复后短暂重放的问题。最新离线汇总为 25 tests / 0 failures / 1 skipped；
本文件下面的 23 项为上一轮历史记录。生产 TDT/环境快照/最终执行连接与 HardwareInterface 仍未全部完成。

## 基线与工作区

- 远端开发分支 `audit-p0-tf-costmap`：`55a2115b9ed1261b26b4eb4aeaee3282560617b2`。
- 远端 main：`7e2afa126988864ff10b5e6e8450f269eb7d3e8e`。
- Windows `rm_nav_v2` 是文件镜像，没有 `.git`。没有重新初始化或覆盖远端仓库。
- 首先尝试 Git 网络读取，现有 Git 代理 `127.0.0.1:1052` 不可用；直连超时，7890 代理重置连接。
  因此本轮 `git fetch` 未成功。用官方 GitHub 连接器读取两个分支及实际修改文件，逐项核对基线。
- 连接器账户是 `Ace-Monaka-Miya`，不是用户要求的 `kswlt`；不使用该账户创建提交。
- 当前只修改本机镜像。没有连接物理串口、启动原导航、修改车辆代码或发出实车运动指令。

## 已修复代码

### 轨迹契约与端点生成器

文件：`src/rm_nav_planning/include/rm_nav_planning/timed_trajectory.hpp`、`minimum_jerk.hpp`、
`src/rm_nav_planning/src/minimum_jerk.cpp` 及对应测试。

- 原契约只检查非空、版本和过期；现在拒绝 NaN/Inf、负 clearance、非递增时间、未来生成时间和终点超过有效期。
- 原自动时长只按距离/速度估计，会违反加速度和 jerk；现在按五次曲线导数解析极值分配时长，
  平移与 yaw 分别约束。显式不足时长拒绝，粗采样也不能绕过连续时间约束。
- yaw 使用常数时间的最短角归一化，避免极大有限输入导致 while 长循环。
- 限制采样总数并检查 int64 时间戳溢出。失败时不留下部分输出；调用方仍必须检查返回值，不能重放旧结果。
- Release 下明确开启 assert，避免原 assert 测试被 NDEBUG 消除。

这仍然只是端点五次曲线，不能当作 PDF 的 TDT OSQP corridor 后端通过。
追加的 yaw 限制默认值是 1 rad/s、2 rad/s²、10 rad/s³；原四项聚合初始化保持可编译，
但角速度不再隐式复用平移限制，调用方应显式配置真实执行器限制。

### SweptFootprint 与 polygon

文件：`swept_footprint_validator.hpp/.cpp`、新增 `polygon_footprint_validator.hpp/.cpp` 和测试。

- 原固定四步、直接线性 yaw 插值会漏过障碍并跨 ±π 绕远；现按步长自适应细分并使用最短角。
- 复用 Boost.Geometry 的 polygon 有效性和 polygon/rectangle 相交查询，没有自研通用几何库。
- 检查整个 polygon 与栅格相交，包括 polygon 内部障碍；未知、越界、非法栅格和自交轮廓拒绝。
- 步长结合分辨率和轮廓半径；障碍格按采样间最大轮廓运动量扩张，保守覆盖采样间运动。
- 栅格上限 4000000 格，查询预算 4000000 格，采样预算 100000；超限返回失败。
- 调用方必须保证轨迹/栅格 frame、版本和时间一致。这是已测试的库，尚未接到 Nav2 执行路径。
- 保守扩张可能使窄道被拒绝，需结合真实 footprint 和分辨率验收，不能把保守拒绝当成规划器故障。

### MapBundle 发布

文件：`tools/publish_map_bundle.py`、新增 `tools/test_publish_map_bundle.py`。

- 原端点 PCD 将未观测格全部写成 free=255；现在写 unknown=205，只有高度范围内的端点格写 occupied=0。
- 补充可由 Nav2 map_server 读取的 `maps/map.yaml`，引用相邻 `occupancy.pgm`、resolution 和 origin。
- 地形参考保存全部观测的 minimum Z，独立于障碍高度筛选；明确 `traversability_verified=false`。
- 复用 draft 生成器的严格 binary XYZ PCD 验证，拒绝 WIDTH/HEIGHT/POINTS 不一致；最多 1000000 点。
- 补充高度参数有限性、栅格内存上限、路径中间组件 symlink 拒绝、staging 写入失败回滚。
- Bundle 版本涵盖全部地图产物，改变 resolution/高度过滤/terrain 会改变版本。
- 仍须显式 verified 官方对齐输入；测试使用 TEST_ONLY 来源，没有生成正式场地地图。
- 发布 manifest 明确 `navigation_ready=false`、`free_space_evidence=unavailable_endpoint_only_pcd`。
  localization 可用 PCD 不等于 Nav2 获得了可通行区域。需要真实射线观测或官方可通行 mask 才能补齐 free-space。
- 默认障碍高度 `[0.15, 2.0]` m 只是工具默认参数，不是本车标定；生产使用必须显式设置。

## 本机实际执行结果

环境：Windows 主机 WSL2 Ubuntu-24.04，已有 ROS 2 Jazzy、colcon、Eigen、PCL/Boost。
源码来自 Windows 镜像，构建/安装位于 `/root/adam_nav_local`，没有在 Windows 原源码目录生成 build/install。

首次全量构建实际执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /root/adam_nav_local
colcon build --base-paths /mnt/c/Users/Admin/Desktop/新导航框架/rm_nav_v2/src \
  --executor sequential --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=Release
```

结果：9 个包成功；`rm_nav_hardware` 缺 `hardware_interfaceConfig.cmake`，
`rm_nav_registration` 缺 `small_gicpConfig.cmake`；其下游 3 个包未处理。
本机也未安装 Nav2、robot_localization、GTSAM/完整注册依赖与 LIO overlay，不能报全量构建通过。
WSL 的旧代理地址及 DNS 不可用，暂未能下载补齐依赖。诊断临时创建的 resolv.conf 已移除，恢复原先不存在的状态。

可重复的离线子集验收：

```bash
# WSL Ubuntu-24.04；默认产物 ~/adam_nav_local_offline
bash tools/local_build_test.bash offline
# 重用本轮构建目录：
RM_NAV_LOCAL_BUILD_ROOT=/root/adam_nav_local bash tools/local_build_test.bash offline
# 依赖补齐后，全量验收不会自动跳过失败包：
bash tools/local_build_test.bash full
```

脚本执行 colcon build、colcon test、colcon test-result --verbose 及 MapBundle 单测/smoke。
本轮结果：

| 验证 | 结果 | 边界 |
| --- | --- | --- |
| 离线子集构建 | 9 包通过 | 不包含 registration/hardware/localization/mapping/bringup |
| colcon test-result | 23 tests，0 errors，0 failures，1 skipped | 含 Omni ROS 适配回归；不是实车测试 |
| 轨迹包 | 5 CTest 通过 | 包含连续时间限制、最短 yaw、内部 polygon 障碍、未知和越界 |
| MapBundle 回归 | 8 项通过 | 合成 TEST_ONLY 数据；含版本变化和回滚 |
| MapBundle smoke | 通过 | 原 smoke 断言文件发布，不证明真实场地地图可导航 |
| 实时 MID-360 / MCAP / Nav2 完整闭环 | NOT TESTED | 无硬件连接及完整依赖 |

完整输出保留在 `/root/adam_nav_local/log`、`/root/adam_nav_local/reports`。
Windows 可通过 `\\wsl.localhost\Ubuntu-24.04\root\adam_nav_local\reports` 查看。
报告副本及待提交补丁在本地 `log/development_20261013`，不把日志和合成地图混入生产地图。

## 仍需完成的代码，不是仅剩实车调参

1. **TDT 上游接入**：直接复用 `tdt-nav-kit/src/MinimumSnapOsqp`、OSQP/OsqpEigen，配置五阶和三阶导数目标；
   接 Nav2 Path、当前速度、时间分配、corridor、迭代修复，输出版本化 TimedTrajectory。
2. **轨迹执行连接**：当前 Omni 适配器是速度输入，尚不是 TimedTrajectory 的横向/姿态跟踪器；
   接入 polygon/新轨迹验证、版本改变取消、失效停车并保留 MPPI 回归。
3. **HardwareInterface**：`rm_serial_hardware_interface.cpp` 仍是骨架，feedback_valid 没有真实输入路径、read/write 没有实际桥接。
   my_serial_py 必须是唯一串口所有者；其 RX 协议没有轮速/底盘 twist/停车反馈，不能伪造 state interfaces。
   暂缓轮速时可保留现有纯前馈适配，不得将其称为 HardwareInterface 闭环验收。
4. **MapBundle 完整交付**：官方对齐数据、free-space 观测/官方 mask、可靠地形、真实地图版本加载与冻结定位联调。
5. **建图审计遗留**：500/5000 关键帧容量契约、回环 overlap 搜索性能、优化后残差/形变验收仍需实现和真实回归。
6. **系统级验收**：补齐本机依赖后运行全量构建/测试；再完成公开 rosbag 和真实动态数据、恢复、串口停止语义与长期资源测试。

## Git 状态

没有新 commit SHA，未推送，未合并 main。原因是 CLI 网络不可用且可用连接器身份不符合 kswlt 要求。
本轮不会通过错误账户提交，也不会伪造 git fetch、push 或 ls-remote 成功。
已核对远端基线的修改按轨迹、地图和文档/测试入口分别保存 patch。
恢复 Git CLI 网络及 kswlt 凭据后，在正常 clone 内先 fetch/status，审查并分步应用补丁，
中文 commit，push feature branch，再执行 git ls-remote 核实。
