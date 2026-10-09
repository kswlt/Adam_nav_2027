# VGICP 相邻约束、GTSAM 与原始关键帧重建

2026-10-09，asus / Jazzy，固定上游库实际构建运行。
本步实现离线相邻帧建图链，并增加回环候选预算与双向质量验收接口；当前仍未把回环边接入 GTSAM，
也未完成官方坐标对齐、动态点清理和 MapBundle 发布。

## 固定依赖与运行

GTSAM [稳定版 4.2.0](https://github.com/borglab/gtsam/releases/tag/4.2.0)，提交
4f66a491ffc83cf092d0d818b11dc35135521612，BSD-3-Clause。
不修改上游源码，使用系统 Eigen，关闭 unstable/Python/TBB；当前机器约 7 GiB RAM，
以 Release -O1、单编译进程构建。用户目录 RPATH 同时解决 GTSAM 的 metis-gtsam 运行依赖。
small_gicp 继续使用已固定的 fa0cfc983417c044b6036f71dfa448038bb9c820；
VGICP 调用真实 GaussianVoxelMap 接口，未将普通 GICP 改名冒充。
构建还使用已有系统 Boost 和 OpenSSL/libcrypto。

```bash
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_gtsam.sh
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 CMAKE_PREFIX_PATH=/home/asus/nav_deps/install:${CMAKE_PREFIX_PATH:-} colcon build
source install/setup.bash
# 先停止 keyframe_recorder，保留原始 session。
ros2 run rm_nav_mapping offline_graph_optimizer /absolute/session /absolute/new_output
# 显式启用有界候选、双向 KISS/GICP 和已验收回环边。
ros2 run rm_nav_mapping offline_graph_optimizer /absolute/session /absolute/new_loop_output --enable-loops
```

只接收 2–500 帧连续编号的完整 session；该初版上限用于约束批量优化规模。
采集器仍可记录更多帧，超过 500 的 session 本工具会整体拒绝，不静默截断。
输出目录必须不存在，每条相邻边都必须合格，不合格时整个图拒绝输出，
不会插入未标注的纯里程计替代约束。优化入口为单独离线进程，不在导航输出回调中运行。

新记录节点持有 recording.lock 的独占 flock，优化器获取共享锁失败会拒绝活动会话。
旧版 schema=1 无锁归档仍兼容，操作者必须确认停止记录。
拒绝 partial、错误 schema/frame/CDR 大小、symlink、缺号/重复、非单调源时间/序号、
混合标定/来源/传感器/参考点、非法 SE(3)/原点/XYZ 和逐点时间。
单帧最多 200000 点、32 MiB CDR、64 KiB 元数据；原始 CDR SHA256 在配准/重建两次读取间复核。

## 配准与图

归档点已经在 odom，先用 recorded_odom_T_body 的逆变换到各帧的底盘参考点：

`body_i_point = inverse(recorded_odom_T_body_i) * original_odom_point`

目标为前一帧、源为后一帧。VGICP 初值：

`body_i_T_body_j_initial = inverse(recorded_odom_T_body_i) * recorded_odom_T_body_j`

点预处理体素 0.05 m，Gaussian voxel 0.25 m，单线程确定性预处理；
上游真实 LM 优化，已收敛驻点数值问题才用同一目标的真实 GN 校验步骤。
通过点到原始目标点的最近邻距离报告米制 RMSE，不将 Mahalanobis 目标当作米。
初值相邻平移最多 2 m、旋转最多 1 rad；相邻边门限：
收敛、至少 50 inliers、比例 ≥0.6、RMSE ≤0.15 m、Hessian 特征值比 ≥1e-6、
相对初值修正平移 ≤0.5 m、旋转 ≤0.3 rad。
这些是当前软件初值，尚未通过实车数据标定；重复结构仍可能误配准。

图由真实 GTSAM Pose3 prior + BetweenFactor 构造，首帧以原始位姿锚定（sigma 0.001）。
prior 用于固定坐标自由度，不代表首帧已达到毫米级测量精度。
GTSAM 切空间顺序为旋转 xyz、平移 xyz；相邻因子固定 sigma 为 0.03 rad / 0.05 m，
属于显式调参，不把配准 Hessian 的逆伪装成已标定协方差。
真实 Levenberg–Marquardt 最多 50 次，检查有限位姿、目标不升高及相邻链最终残差上限。
当前只有相邻链：它能使用配准约束改正里程计初值，但没有回环就无法约束全局累计漂移，
不能据此宣称完整 SLAM 或长期全局一致性已经完成。

## 回环候选与验收接口（当前阶段）

`LoopCandidateManager` 现在具有显式且有界的配置：最小时间间隔、最小关键帧 ID 间隔、最大空间距离和最多候选数。
候选按空间距离、时间间隔和 ID 确定性排序，超出预算直接截断。默认最小时间间隔 10 s、最小 ID 间隔 3、
最大距离 5 m、最多 20 个候选。候选只是搜索入口，不代表回环成立。

`loop_validator.hpp` 只接受真实 `RegistrationResult`，正反向都必须收敛并达到 inlier/比例、RMSE、Hessian 条件、
双向重叠和三维几何阈值；修正不能超过轨迹先验，并检查正反向变换往返误差。任一条件失败即拒绝，
避免把重复结构或单平面匹配直接写入图。
当前 `pose_graph.cpp` 已支持单独的 validated loop edge 输入：回环边必须显式设置
`loop=true`、`validated=true`，拥有有限的正定噪声和 Huber 参数；未验收或被伪装成相邻边的输入会拒绝。
回环因子使用 GTSAM Huber 鲁棒核，优化后至少要求目标函数有限且不增加；含回环图不再错误地要求相邻链的近零残差。
离线优化器已在 `--enable-loops` 下从关键帧生成候选，执行真实 KISS 正反向配准、重叠率和几何检查，
只把 `LoopValidationResult.accepted` 的边交给 GTSAM；默认不启用回环，便于对照相邻链结果。

回归覆盖候选排序/预算、非法配置、正确双向回环、残差过大、重叠不足、几何退化、正反向不一致和未收敛输入。
`tools/smoke_offline_mapping.py` 现在还会运行 `--enable-loops` 路径，确认无候选时输出确定性的空
`loop_edges.json`，并验证原始归档、相邻图和地图重建不受影响。该测试证明的是接线和门逻辑，不证明真实场地回环率。

运行时默认 use_voxelized_target=false，仍使用原有 GICP；
内部 MAPPING_VGICP 枚举不向运行时定位 ROS 消息发布。

## 重建与产物

重新读取每个原始关键帧，而非配准降采样云或已拼接在线地图：

`rebuilt_point = optimized_T_body * inverse(recorded_odom_T_body) * original_odom_point`

只做确定性 0.05 m 体素去重，最多 1000000 个输出体素；尚无时间持久性动态点清理。
原始点数据与元数据不写回；CDR SHA256、参考点和标定均复核。
产物为 optimized_poses.json、adjacent_edges.json、loop_edges.json、input_hashes.json、二进制 XYZ rebuilt_map.pcd。
优化位姿保留来源/标定/参考点；边记录实际算法、变换和质量；输入表记录 CDR SHA256。
坐标标记 mapping_odom、official_alignment_applied=false，不自动加载到 match 模式或发布 map→odom。
源参考原点仍在原始归档，XYZ PCD 不保留逐点时间/射线信息，不能作为射线清障证明。

输出先写入新的 sibling partial 目录，文件关闭后逐文件/目录 fsync，
Linux renameat2(RENAME_NOREPLACE) 提交，再 fsync 父目录；不覆盖现有输出。
写入失败可能留下 partial；需检查后处理。若重命名已成功但最终 fsync 失败，
可能存在未成功返回的输出目录，不能自动把它当作已发布 MapBundle。
本工具不模拟突然断电，保证依赖本地文件系统和存储设备的 fsync 行为。

## 验收

```bash
./build/rm_nav_registration/small_vgicp_backend_test
./build/rm_nav_mapping/pose_graph_test
python3 tools/smoke_offline_mapping.py
ROS_DOMAIN_ID=102 python3 tools/smoke_mapping_capture.py
ROS_DOMAIN_ID=103 python3 tools/smoke_small_point_lio.py --with-state --with-mapping
```

真实 VGICP 已知变换方向、质量、驻点与非法输入通过；
真实 GTSAM 五帧不一致图目标下降、优化位置、单帧锚与非法因子拒绝通过。
离线验收构造六帧具有已知真值的三维几何归档，主动给初始里程计加入位置/转角漂移；
通过实际可执行程序生成真实 VGICP 因子、GTSAM 优化和原始点云 PCD。
本次末端初值偏差约 0.269 m，优化后约 0.000238 m；图目标 3.78938→约 7.39e-30。
这是理想合成数据结果，不是实车精度承诺。
检查原始输入不可变、输出哈希、活动会话/已有输出拒绝，错误标定/时间/缺帧/CDR 截断/
非刚体位姿/原点/不重叠几何/partial 会话均不生成地图。
实际采集节点、Point-LIO/EKF/观测/GICP/归档与运动门回归另使用明确的合成硬件输入。
日志与样例保留在 asus 工程 log/offline_mapping_*，生成数据不上传 Git。

下一步接入 KISS 候选粗配准 + 精化 + LoopValidator，再允许回环因子进入图。
之后完成地图清理、官方坐标对齐、MapBundle 原子发布和真实 MCAP/实车验收。
