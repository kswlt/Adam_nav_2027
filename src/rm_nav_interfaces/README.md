# rm_nav_interfaces

Custom messages and actions for RM Nav V2.

轨迹执行新增 `TrajectoryPoint`、`TimedTrajectory`、`TrajectoryEnvironment`。
`TimedTrajectory.header.stamp` 是执行时间原点，points 中时间严格递增且从 0 开始；
位姿、速度均在 header.frame_id，yaw/wz 单位 rad/rad/s；最多 200000 个点。
trajectory_id 在同一规划生产者会话内严格递增；失败、取消、版本改变之后不能重放旧 ID。
`valid_until` 必须覆盖轨迹终点；map/environment 版本必须与原子环境快照匹配。
`TrajectoryEnvironment.grid.header` 携带实际栅格的源时间/坐标系，快照同时绑定地图/环境版本，
不能把任意旧栅格换个版本字符串充当新地图。见 [轨迹验收](../../docs/TRAJECTORY_TRACKER_ACCEPTANCE.md)。

已生成 `RegistrationEstimate` ROS 消息，携带地图版本、源观测时间、frame、
map_T_odom、方法、质量、Hessian、点数和迭代/耗时数据。
字段约定见 `msg/RegistrationEstimate.msg`，运行边界见
[冻结地图定位验收](../../docs/frozen_map_localization_acceptance.md)。
