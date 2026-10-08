# rm_nav_frames

本包定义 RM Nav V2 的 TF 所有权和标定输入。第一阶段只安装配置和约束，不自行发布重复 TF。

## TF ownership

```text
map -> odom                 rm_nav_localization / MapOdomManager
odom -> base_footprint      robot_localization / ChassisState
base_footprint -> chassis   static calibration
chassis -> big_gimbal_yaw   gimbal encoder adapter
big_gimbal_yaw -> sensors   CalibrationBundle
```

重定位只能更新 `map -> odom`。Point-LIO 的连续 odom 不得被重置。
