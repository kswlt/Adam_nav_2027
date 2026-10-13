# 固定 MID360 与 legacy TF 验收

2026-10-13，在本机 WSL Jazzy 检查发现 `local_state` 的 legacy 分支同时启动
legacy 静态 footprint→chassis 和动态投影器；其 base_link 别名父节点也与状态桥的静态查询语义不符。
此外，固定传感器分支缺失，使底盘状态桥始终等待不存在的云台编码器。

修复：legacy 单独联调仍保留静态安装；由 local_state 调用时明确关闭静态 chassis 发布，
别名使用 chassis→base_link。新增 `sensor_mount: fixed_chassis`，只发布 chassis→IMU→LiDAR
与 chassis→base_link 静态 TF，状态链不要求不存在的编码器，也不发布虚拟健康数据。
动态云台默认要求真实编码器的行为保持不变。

`vehicle_164_fixed_mid360.yaml` 使用用户提供 chassis→LiDAR `[0,0.08,0.20]`、正装；
固定上游内部 IMU→LiDAR `[-0.011,-0.02329,0.04412]`，推导 chassis→IMU
`[0.011,0.10329,0.15588]`。旋转为恒等；组合后恰好得到用户安装位置。
状态 `user_provided_factory_internal`，`field_calibration_verified=false`，必须显式 `allow_user_mount:=true`。
没有把工厂示例或用户安装声明改写为实测联合标定通过。

构建：全 14 包 colcon build 通过；colcon test/result 为 49 tests、0 errors、0 failures、1 skipped。
测试：`ROS_DOMAIN_ID=176 python3 tools/smoke_fixed_mount.py` 与
`ROS_DOMAIN_ID=177 python3 tools/smoke_fixed_mount.py --legacy` 通过。
合成 raw IMU 位姿输入进入实际 TF/resolver/robot_localization，保留完整倾斜姿态，
验证安装位移只应用一次、动态 chassis 唯一 authority、正确 base_link 父节点、无虚拟编码器、断流健康失效。
动态云台回归 `smoke_local_state.py` 同轮通过。

实车安装方向、参考高度与内部参数仍需现场联合验证；本轮没有连接或改动原车进程。
