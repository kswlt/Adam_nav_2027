# 当前实车建图测试会话

2026-10-22用户要求测试建图→导出→离线优化→导航。已在ASUS新独立release/runtime启动实际传感器链；未启动导航任务、candidate或physical，用户遥控行驶。

- release：`/home/asus/nav_2027_vehicle_config_164/releases/20261022_fit153_console`
- runtime：`/home/asus/nav_2027_vehicle_config_164/runtime_20261022_fit153_console`
- 关键帧：`keyframes_fit153cm_debug_v3/session_1792670869656779404_3581137020`
- calibration_id：`vehicle164_yaw_axis_fit153cm_debug_v3`
- 原始录包：本次20:07:48启动，参数600秒（10分钟上限）；关键帧采集/在线优化不会随录包超时停止。
- 运行组件：driver、state、foxglove、bag、capture、online。
- 当前仍是mapping_odom调试建图，不是发布后的导航地图。

初始化前只读IMU检查6秒，1037个新鲜样本，gyro模长P95约0.0128rad/s，加速度模长中位数0.9983g、标准差0.00381g，符合低运动初始化条件；不作为独立静止真值。

启动后15秒检查：真实点云、LIO、去畸变云、参考位姿、滚动子图和累计预览都有输出，输入/点云/参考状态/子图健康为true；LIO年龄P95约11.6ms。TF读到预期父子关系，但当前rclpy消息信息没有提供GID，未独立验证每条TF的唯一authority，不能把probe的passed解释为所有TF authority已验收。完整结果见 [启动探针](evidence/mapping_start_20261022/mapping_start_probe_20261022.json)。

随后online状态：45帧、无积压、fault=false；算法已接纳部分回环边。此时主要是启动及重复观测，不能把接纳边数当作已完成真实运动闭环验证。

已向用户提示可以开始遥控低速绕场、返回起点、不抬车。Foxglove地址 `ws://192.168.1.164:8765`；优化累计图用mapping_online.json / mapping_odom，快速累计预览用sensor_reference_mapping.json / odom。

下一步：用户确认完成后，先停止bag和capture，确认写盘结束；保留online快照导出检查包，再做独立离线优化、地图审查/发布和定位测试。不要直接清空会话、重置odom后继续追加或拿draft冒充正式MapBundle。物理自主导航输出需在地图和定位检查后单独启动。

本轮增加只读IMU探针和可选录包时长参数，默认180秒不变，允许10～3600秒。本次运行目录的组件脚本已同步该参数；源码release基线仍为8a9a9b1，变更通过中文提交另行推送。Bash语法及Python编译检查通过，未重复完整colcon测试。
