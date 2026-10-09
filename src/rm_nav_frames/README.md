# rm_nav_frames

本包定义 RM Nav V2 的 TF 所有权和标定输入，不自行发布重复 TF。
`local_state_calibration.template.yaml` 给出动态云台状态链所需测量字段；默认 draft 和
空数值，必须实测并显式确认后才能用于启动。运行 TF 发布者位于 bringup 的 gimbal_tf。

## TF ownership

```text
map -> odom                 rm_nav_localization / MapOdomManager
odom -> base_footprint      robot_localization / ChassisState
base_footprint -> chassis   static calibration
chassis -> big_gimbal_yaw   gimbal encoder adapter
big_gimbal_yaw -> sensors   CalibrationBundle
```

重定位只能更新 `map -> odom`。Point-LIO 的连续 odom 不得被重置。
