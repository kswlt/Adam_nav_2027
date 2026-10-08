# rm_nav_localization

LIO adapter, chassis resolver, state estimation and relocalization.

实际 ROS 入口：`frozen_map_matcher` 调用 small_gicp 匹配冻结地图与 odom 子地图；
`map_odom_manager` 验收质量、时间、版本与修正幅度后独占发布 map→odom。
启动、话题和故障验收见 [冻结地图定位验收](../../docs/frozen_map_localization_acceptance.md)。
输入子地图生成、LIO、EKF、KISS 和健康状态到运动许可的接入仍待完成。

The first implemented component is `ChassisResolver`. It applies the PDF-defined relation:

```text
world_T_chassis = world_T_lidar * inverse(chassis_T_lidar(t))
```

It does not filter, publish TF, or invent timestamps. Those responsibilities stay with the Sensor Hub, `robot_localization`, and the TF authority layer.

`CandidateRegionGenerator` limits global recovery search to `max_speed * lost_time + safety_margin`, clipped to official field bounds. KISS and GICP adapters consume this region but do not own its policy.

`MapOdomManager` is the sole state holder for `map->odom`; it rejects `UNVERIFIED`, `CANDIDATE`, and `REJECTED` registration results. Point-LIO odometry remains continuous and is never reset here.

`LocalSubmapBuilder` maintains the configured rolling time window and transforms each input frame into `odom`. It reports timestamps and point count for registration quality checks.

`RelocalizationStateMachine` 当前仅提供状态/许可辅助接口；尚未驱动真实恢复流程或控制输出。
