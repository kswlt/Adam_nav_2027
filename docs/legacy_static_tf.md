# 原版静态 TF 联调配置

`legacy_static_tf.launch.py` 复用了原版 Adam 工程
`rm_static_tf/launch/static_tf.launch.py` 中的三个导航相关静态变换：

```text
base_footprint → chassis       (0, 0, 0.076)
chassis        → base_link     (0, 0, 0)
chassis        → front_mid360  (0.16, 0, 0.18; 四元数 [1, 0, 0, 0])
```

启动方式：

```bash
ros2 launch rm_nav_bringup legacy_static_tf.launch.py
```

这是用户指定的原版静态外参复用，只用于 Foxglove、Costmap 和点云转换联调。
它不修改 `CalibrationBundle`，不解除 `local_state.launch.py` 的 verified 标定门，
也不代表当前机器人已经完成实测外参验收。后续获得实测数据后，应替换为正式
CalibrationBundle，并停止使用本启动文件。
