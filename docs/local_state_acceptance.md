# 动态云台底盘状态链

链路为上游 small_point_lio 原始 IMU 位姿 → sensor_pose_resolver →
上游 robot_localization EKF；编码器节点仅负责标定与时间戳 TF，不自研 EKF。

## 标定与启动

```bash
source /opt/ros/jazzy/setup.bash
source /home/asus/nav_deps/install_lio/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup local_state.launch.py \
  calibration_file:=/path/to/measured_bundle.yaml \
  lio_params_file:=/path/to/driver_and_filter.yaml
```

参考 `src/rm_nav_frames/config/local_state_calibration.template.yaml` 的字段。
模板为 draft、数值未测量，启动必须拒绝；不得把测试外参当实车默认配置。
verified 只是显式配置声明，需要实际测量与审核，不是程序自动认证的标定精度。
同一个 bundle 提供静态 TF、云台零位及 IMU→LiDAR 外参，覆盖上游参数文件中外参，
关闭上游在线外参估计，避免两套标定真值。

TF：base_footprint→chassis（静态）→big_gimbal_yaw（编码器动态）→IMU→LiDAR（静态）；
base_link 是 chassis 参考点的静态别名。
上游 LIO 不发布 TF，resolver 也不发布 TF，唯一 odom→base_footprint 来自 EKF。
输出 `/state/chassis` 是完整位姿/速度/covariance 的 Odometry；参考点桥已接入
Nav2 `/odom`，本地健康约束任务与输出门，见 [状态桥验收](state_observation_acceptance.md)。
不能直接用 EKF 持续预测的输出时间戳证明传感器仍健康。

## 编码器和解算规则

`/hardware/gimbal_joint_states` 使用 JointState，header frame 为 chassis，
配置的 joint position 是绝对编码器角度，单位 rad，时间为真实观测时刻。
支持标定的符号、零偏和旋转中心；不使用 `/contact_angle`。
两份连续观测间隔不超过 0.1 s 后才健康；源时间/接收时间不超过 0.2 s。
无编码器不发布假零角动态 TF，非法或断流使健康 false。

resolver 按原始位姿的精确时间查询 TF，由 tf2 插值，不使用 latest-time 退路：
精确时刻 TF 最多等待 50 ms 到达，等待后重新检查源时间与编码器新鲜度。
`odom_T_body = odom_T_imu * inverse(body_T_imu(t))`。
保留完整 SE(3)、平移杠杆臂、pitch/roll 与源时间；拒绝过期、重复、错 frame、
非有限或非正定 covariance。通过杠杆臂 Jacobian 传播位姿协方差。
当前传播假设标定和编码器角度精确，尚未增加其测量不确定度，实车必须补充评估。
`/state/lio_healthy` 与 `/state/gimbal_healthy` 有独立心跳和超时，不能当作硬件停车反馈。

EKF 默认融合 resolved LIO 的六维位姿；wheel odom 与独立 chassis IMU 默认关闭，
只有真实反馈存在、frame/量纲/covariance 已核对后才能打开
`enable_wheel_odometry` / `enable_chassis_imu`。没有伪造这些输入。

## 软件验收

```bash
ROS_DOMAIN_ID=99 python3 tools/smoke_local_state.py
```

测试关闭 LIO 节点，使用明确标注的合成原始位姿与编码器观测，启动实际云台 TF、
resolver 和 robot_localization。真实 Point-LIO 本身另见上游验收，不能混称本测试
验证了真实 LiDAR/IMU 的运动数据。
检验带平移偏心传感器旋转时底盘仍固定、完整 pitch/yaw、杠杆臂 covariance、
实际 EKF 的输出 frame/位置、无重复 TF，以及错误年龄/frame/NaN/covariance、
编码器和位姿断流拒绝。日志为 `log/local_state_smoke.log`。
2026-10-09 在 asus/Jazzy 完整通过上述软件验收。
后续仍需实车标定、硬件时钟同步与真实 LIO/编码器回放。
