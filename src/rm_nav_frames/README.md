# rm_nav_frames

本包定义 RM Nav V2 的 TF 所有权和标定输入，不自行发布重复 TF。
`local_state_calibration.template.yaml` 给出动态云台状态链所需测量字段；默认 draft 和
空数值，必须实测并显式确认后才能用于启动。运行 TF 发布者位于 bringup 的 gimbal_tf。

## TF ownership

```text
map -> odom                 rm_nav_localization / MapOdomManager
odom -> base_footprint      base_footprint_projection (planar projection)
base_footprint -> chassis   base_footprint_projection (dynamic 6DoF residual)
chassis -> big_gimbal_yaw   gimbal encoder adapter
big_gimbal_yaw -> sensors   CalibrationBundle
```

`robot_localization` 保留完整 `odom -> chassis` 状态到 `/state/chassis`，不直接发布 TF；
`base_footprint_projection` 将同一时刻的 chassis 状态分解为平面 `odom -> base_footprint`
和动态 `base_footprint -> chassis`，因此 z/roll/pitch 不会污染平面导航 frame。
重定位只能更新 `map -> odom`。Point-LIO 的连续 odom 不得被重置。
