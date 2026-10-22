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

## 本次结束与实际分析

用户确认目前车辆只能云台旋转、已完成旋转。因此本次是**底盘静止、偏心雷达随云台旋转**，不是绕场数据。bag和capture已正常SIGINT退出，录包日志确认缓存写盘完成；driver/state/foxglove/online保留运行。physical和navigation未启动。

原始MCAP：`runtime_20261022_fit153_console/bags/run_20261022_200748`，327.595秒、98,097条消息，约2.5GiB。关键帧160帧，约46MiB。在线revision160、累计10,952点、150条接纳回环边、1条拒绝、无积压、fault=false；接纳边主要属于静止/旋转重复观测，不等于完成150次实车绕行闭环。

| 项目 | 本次记录结果 |
| --- | --- |
| LIO IMU参考点 | 3,201帧，XY相对起点最大38.59cm，末尾5.30cm |
| 实际发布的补偿轴参考点 | 3,199帧，XY相对起点最大6.68cm，末尾0.857cm |
| 从LIO重新计算的3D轴参考点 | 最大XY变化6.35cm、末尾0.971cm、z范围1.98cm |
| 轴参考点实现复核 | 同源时间戳匹配3,199帧，重算与发布XY最大差0 |
| 雷达yaw | 展开角范围378.77°、末尾相对起点378.09° |
| IMU | 65,384帧，角速度模最大7.567rad/s（约434°/s） |
| 源时间戳 | LIO/参考位姿/IMU未发现非单调 |

两种轴参考统计起止样本不同，故最大值/末尾不完全相同；不能把这点解释为公式不一致。参考位姿的yaw与z按当前平面导航定义为0，不能据此判断真实传感器没有旋转/高度变化。以上全是估计值，没有独立运动真值，不能声称6.68cm全部为LIO漂移或已验收机械标定。

### 在线导出成功，离线优化拒绝

在线快照导出至 `runtime_20261022_fit153_console/review_rotation_20261022`，包含PCD、优化轨迹CDR、PGM/YAML、terrain_reference与manifest。坐标仍为mapping_odom，状态draft_mapping_review；高度切片[-0.10,1.50]m只是检查范围，未测地面。栅格1,467个占据单元、211,249个未知单元、0个自由单元，**不能用于导航**。

实际离线命令：

```bash
ros2 run rm_nav_mapping offline_graph_optimizer \
  "$NAV_RUNTIME/keyframes_fit153cm_debug_v3/session_1792670869656779404_3581137020" \
  "$NAV_RUNTIME/offline_rotation_20261022" --enable-loops
```

返回1，原因：第139→140帧VGICP RMSE=0.153856m，超过0.15m门限（inlier_ratio=0.795343）。这是确定存在的拒绝条件；尚未排除其他未打印的增量条件。程序未插入纯里程计桥接、未产生离线优化输出目录，保留完整160帧原始数据。

失败帧时间戳1792671158936748504ns。其前后各1秒窗口内LIO yaw变化350.89°，IMU角速度峰值7.567rad/s；峰值仅比失败帧早0.156秒。整包角速度模>0.2rad/s仅约1.87秒，运动主要集中在短促快速旋转中。高速旋转与配准失败在时间上重合，但还不能断言具体是去畸变、重叠率、场景动态还是配准模型造成。没有放宽门限，也未把在线接纳当作离线通过。

只读诊断脚本 `tools/analyze_rotation_session.py` 在ASUS运行通过；它不启动ROS节点、不回放、不发控制。实际证据见 [统计JSON](evidence/rotation_20261022/rotation_focus_analysis_20261022.json)、[离线失败日志](evidence/rotation_20261022/offline_rotation_20261022.log)、[导出manifest](evidence/rotation_20261022/map_bundle.json)。本轮仅新增诊断/文档，未重跑完整colcon。

下一次建议单独录：两端静止各10秒，中间以约15～30°/s匀速转一圈，避免快速起停；对比慢转与本次快转的轴心轨迹和同一配准门限。底盘可平移后再采绕场数据、验证自由空间并发布本地场地地图，之后做重定位与导航行驶验收。不要清空现有会话或把draft提升为正式MapBundle。

本轮增加只读IMU探针和可选录包时长参数，默认180秒不变，允许10～3600秒。本次运行目录的组件脚本已同步该参数；源码release基线仍为8a9a9b1，变更通过中文提交另行推送。Bash语法及Python编译检查通过，未重复完整colcon测试。
