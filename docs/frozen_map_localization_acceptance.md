# 冻结地图 ROS 定位链验收

2026-10-09，在 asus / ROS Jazzy 构建并运行真实 ROS 节点。
使用锁定的 small_gicp 1.0.1，没有连接硬件或伪造算法输出。

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup frozen_map_localization.launch.py map_version:=your-map-version
```

运行两个 C++ 节点：`frozen_map_matcher` 与 `map_odom_manager`。
地图版本参数必填，两个节点必须一致；生产环境应使用 MapBundle 版本或内容哈希。
当前版本是会话约束，尚未实现 MapBundle 文件内容哈希校验。

## 输入与输出

| 话题 | 类型 | 约定 |
| --- | --- | --- |
| `/localization/frozen_map` | PointCloud2 | map 坐标，reliable/transient local，首个有效地图冻结 |
| `/localization/odom_submap` | PointCloud2 | odom 坐标，best effort，去畸变完成，stamp 是原始观测时间 |
| `/localization/initial_guess` | TransformStamped | 可选粗初值，parent=map、child=odom；不会直接发布 TF |
| `/localization/estimate` | rm_nav_interfaces/RegistrationEstimate | 地图版本、原观测时间、map_T_odom、配准质量与 Hessian |
| `/localization/map_to_odom` | TransformStamped | 仅接受后更新，transient local；反馈给匹配器作为下次初值 |
| `/localization/healthy` | Bool | 当前定位质量许可，50 ms 发布，接受结果超过 1 s 失效 |
| `/localization/correction_pending` | Bool | 大幅修正候选，当前不能直接提交 |
| `/localization/status_reason` | String | 验收/拒绝/过期原因 |

匹配器用当前初值把 odom 子地图包围盒变换至 map，增加 1 m padding，裁剪冻结目标。
最高匹配频率 5 Hz；默认体素 0.05 m、最大对应距离 0.5 m、两个线程。
PointCloud2 支持 float32/float64 XYZ、组织化行 padding、两种字节序；
拒绝超限点数、存储/字段越界、非有限坐标及绝对坐标超过 10000 m 的点。
同一冻结版本内地图内容发生变化后暂停匹配，需要新版本和新会话。
当前每次配准仍重新预处理裁剪目标；KD-tree 缓存和流式地图块管理尚未实现。

## TF 验收

MapOdomManager 是本 launch 内唯一 map→odom 发布者，启动时没有未经验证的单位 TF。
输入必须满足 map/odom frame、版本、严格递增观测时间及 -0.1～0.8 s 的时间窗口。
质量门限默认：inlier_ratio≥0.5，RMSE≤0.15 m，Hessian 最小/最大特征值比≥1e-6。
大修正门限默认 0.5 m / 0.35 rad，均相对于已发布的 map_T_odom 重新计算，
不会相信输入报文自报的 correction delta。
候选、非法结果和失效观测不改变 TF；定位不健康时保持最近接受的空间变换。
上述参数是调试门限，尚未用场地数据校准。

健康信号现已接入 `motion_gate` 控制输出门；该门还要求任务侧的新鲜运动许可。
软件输出失健康归零已通过测试，当前尚未接入真实硬件，不能宣称实车已经完成停车验收。
大幅重定位仍缺 KISS/独立确认、实车停止确认、轨迹作废和重规划事务。
此处只验证拒绝大修正与保留候选，不自动接受大修正。
odom→base、去畸变子地图生成、冻结地图文件加载、LIO/IMU/轮速均由后续模块提供。

## 验收复现与结果

```bash
ROS_DOMAIN_ID=89 python3 tools/smoke_frozen_map_localization.py
./build/rm_nav_localization/ros_conversions_test
```

脚本启动真实两个节点，用固定随机种子的 2000 点 odom 子地图和冻结地图验证配准。
地图添加 100 个远处点，检查目标裁剪将这些点排除。
已知变换平移 `(0.12,-0.10,0.03)` m、yaw 0.04 rad；
本次平移误差 `0.00001344 m`、RMSE `0.001309 m`，TF 与接受结果一致。
以下故障注入全部通过：

- 启动前没有未验证 TF；质量通过后发布。
- 重复或过期观测、错地图版本、NaN 质量、非法四元数被拒绝且 TF 不变。
- 输入自报 delta=0 的 3 m 大修正仍被判为候选，定位健康为 false。
- 接受结果超时后健康为 false，空间 TF 保持最后有效值。
- 越界存储/过期源点云不生成配准结果。
- 同一版本地图内容变化后停止匹配。

单测另覆盖 float64 大端组织化点云、行 padding、字段与存储越界、点数上限和变换校验。
日志及结果位于 `log/frozen_map_localization_smoke*`。
