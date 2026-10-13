# rm_nav_planning

Nav2 path preprocessing, trajectory data contract and collision validation.

正式 TDT Polynomial 核心已接入，见 [TDT 验收](../../docs/TDT_BACKEND_ACCEPTANCE.md)。
`TdtMinimumJerkBackend` 复用上游 OSQP/corridor/碰撞迭代，并对真实系数计算连续时间约束上界。
先执行 `tools/bootstrap_tdt.sh` 安装固定 OSQP/OsqpEigen；依赖前缀可用 `RM_NAV_DEPS_ROOT` 指定。

This package is part of the RM Nav V2 staged implementation.

`TimedTrajectory` carries map/environment versions and an expiry timestamp. A controller must reject a trajectory after a map correction, environment update, or expiry.

`MinimumJerkGenerator` 生成五次多项式 minimum-jerk 轨迹，采用最短 yaw 角、速度/加速度/jerk
边界和 map/environment 版本绑定。它是独立的端点插值工具，**不等于 PDF 要求的 TDT OSQP corridor 后端**。
TDT 的多段路径、当前速度约束、走廊 QP 和迭代碰撞修复仍需接入上游组件。

生成器用五次曲线导数的解析极值约束全时域，分别限制平移与 yaw。
`duration_s <= 0` 自动分配足够时长；显式时长不足、NaN、采样数超限、时间戳溢出均拒绝。
失败时 `output` 保持原值；调用者必须检查返回值，不能在失败后重新发送旧轨迹。
最多 200000 个采样点；自动时长不承诺满足动态障碍或真实制动要求。

`TimedTrajectory::valid_at` 检查有限值、严格递增时间、生成时间、有效期、版本及轨迹终点期限。

`PathPreprocessor` removes only redundant closely spaced points. It does not replace Nav2 planning or the final swept-footprint collision validator.

`SweptFootprintValidator` samples interpolated translation and yaw poses through an environment query. The query owns the real polygon footprint and costmap/ESDF collision logic.

Yaw 使用最短角插值；默认最大平移步长 2 cm、yaw 步长 0.02 rad，采样预算超限拒绝。
可通过 `SamplingLimits` 依据地图分辨率调整步长。

`PolygonFootprintValidator` 接收同一 frame/version 的占据栅格快照与真实 polygon，复用 Boost.Geometry
判断 polygon 与障碍格相交，覆盖轮廓内部障碍；未知区域、越界、自交 polygon、非法栅格拒绝。
步长由栅格分辨率及轮廓半径确定，障碍格按采样间最大轮廓运动量扩张，保守覆盖采样间运动。
全次查询最多扫描 4000000 个格子，超出预算拒绝。
调用方仍负责栅格时效、frame/version 一致性与地形约束；该库尚未接入 Nav2 执行路径，不能因此宣称控制闭环完成。
