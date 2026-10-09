# 原版静态 TF 联调配置

`legacy_static_tf.launch.py` 复用了原版 Adam 工程
`rm_static_tf/launch/static_tf.launch.py` 中的三个导航相关静态变换：

```text
base_footprint → chassis       (0, 0, 0.076)
chassis        → base_link     (0, 0, 0)
chassis        → front_mid360  (0.16, 0, 0.18; 四元数 [1, 0, 0, 0])
chassis        → front_mid360_imu (同上，仅用于匹配当前 Point-LIO child frame)
```

启动方式：

```bash
ros2 launch rm_nav_bringup legacy_static_tf.launch.py
```

这是用户指定的原版静态外参复用，只用于 Foxglove、Costmap 和点云转换联调。
它不修改 `CalibrationBundle`，不解除 `local_state.launch.py` 的 verified 标定门，
也不代表当前机器人已经完成实测外参验收。后续获得实测数据后，应替换为正式
CalibrationBundle，并停止使用本启动文件。

## LIO/EKF 联调模式

如果需要让 Point-LIO → resolver → EKF 在没有绝对编码器的情况下联调，可显式使用
`local_state_calibration.legacy_adam_static.yaml`，并添加：

```bash
ros2 launch rm_nav_bringup local_state.launch.py \
  calibration_file:=.../local_state_calibration.legacy_adam_static.yaml \
  lio_params_file:=<Point-LIO参数> \
  enable_mid360_guard:=true \
  allow_legacy_static_calibration:=true
```

该文件把原版 `chassis→front_mid360` 拆成 `chassis→gimbal_yaw→imu`，并假设
`imu→lidar` 为单位变换；这是联调假设，不是实测外参。默认参数仍为 `false`，
因此普通启动不会接受该文件。
