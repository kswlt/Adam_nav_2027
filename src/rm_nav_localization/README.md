# rm_nav_localization

LIO adapter, chassis resolver, state estimation and relocalization.

实际 ROS 入口：`frozen_map_matcher` 调用 small_gicp 匹配冻结地图与 odom 子地图；
`map_odom_manager` 验收质量、时间、版本与修正幅度后独占发布 map→odom。
启动、话题和故障验收见 [冻结地图定位验收](../../docs/frozen_map_localization_acceptance.md)。
KISS→GICP 受限恢复会话已接入；不同源点云复核候选，期间健康为 false。
见 [恢复调度验收](../../docs/kiss_recovery_ros_acceptance.md)。健康信号已接到 motion_gate。
实测停车门、Nav2 取消、TF 提交及双清图事务已接入，见
[事务验收](../../docs/recovery_transaction_acceptance.md)。默认关闭，提交后仍等待新规划。
受限自动恢复、提交后稳定确认与新任务放行已接入，见
[任务验收](../../docs/task_supervisor_acceptance.md)。
上游 small_point_lio 已固定版本构建；`sensor_pose_resolver` 按源时刻查询云台 TF，
输出完整底盘位姿及杠杆臂传播后的协方差，交给真实 robot_localization EKF。
见 [状态链验收](../../docs/local_state_acceptance.md)。
输入子地图生成、实测标定、真实驱动/反馈与回放仍待完成。

The first implemented component is `ChassisResolver`. It applies the PDF-defined relation:

```text
world_T_chassis = world_T_lidar * inverse(chassis_T_lidar(t))
```

It does not filter, publish TF, or invent timestamps. Those responsibilities stay with the Sensor Hub, `robot_localization`, and the TF authority layer.

恢复服务按 `max_speed * lost_time + safety_margin` 计算半径，超过配置上限或场地范围拒绝。
matcher 在场地范围内保留候选位置周围的目标特征，并限制点数；不会无约束搜索整场地图。
`CandidateRegionGenerator` 保留为基础接口辅助类；实际 ROS 恢复参数由 MapOdomManager 验证。

`MapOdomManager` is the sole state holder for `map->odom`; it rejects `UNVERIFIED`, `CANDIDATE`, and `REJECTED` registration results. Point-LIO odometry remains continuous and is never reset here.

`LocalSubmapBuilder` maintains the configured rolling time window and transforms each input frame into `odom`. It reports timestamps and point count for registration quality checks.

`RelocalizationStateMachine` 当前仅提供状态/许可辅助接口；尚未驱动真实恢复流程或控制输出。
