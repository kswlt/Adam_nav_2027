# 雷达朝向触发关键帧

基线3d99665，开始前检查工作区并fetch。修复nav_reference方向固定导致纯云台旋转不录关键帧的问题。

新增`rotation_trigger_source=body|sensor`：默认body保持原完整底盘语义；雷达参考部署显式选sensor，从同一原始ObservationFrame.sensor_pose的水平朝向判断旋转阈值。平移仍按原body参考点，归档recorded_odom_T_body仍是精确时间TF，不把雷达旋转塞进nav_reference。原CDR、源时间、500帧/磁盘预算不变，metadata新增rotation_trigger_source说明。

ASUS14包构建通过；colcon test及test-result：57项、0错误、0失败、1可选依赖跳过。实际ROS recorder回归通过：默认模式不因传感器单独旋转触发；sensor模式小角度不触发、纯旋转超过阈值触发；归档body矩阵保持单位姿态，原CDR相等，配额/断流/碰撞原回归保留。C++测试覆盖角度跨±pi及非法角度。

现场入口（新会话，不恢复旧失败session）：

```bash
ros2 launch rm_nav_bringup mapping_capture.launch.py \
  archive_root:=/实际持久化目录 calibration_id:=vehicle164_sensor_axis_factory_v1 \
  body_frame:=nav_reference state_health_topic:=/state/sensor_reference_healthy \
  rotation_trigger_source:=sensor
```

部署脚本capture已选sensor。用户当前不能移动，尚未执行真实云台转动采集，不能把隔离fixture说成实车旋转验收。当前不因此启动串口或自动运动。

ASUS日志：runtime_20261021_sensor_reference/mapping_heading_{build,test,test_result,smoke}.log。原档案与原车工作区保留。
