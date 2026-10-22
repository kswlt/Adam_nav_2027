# 安装关系更正与用户批准的15.3 cm拟合配置

用户2026-10-22明确更正：此前“左16 cm”指**yaw旋转轴→LiDAR测量中心向左16 cm**。此前代码和文档将其理解成“底盘中心→yaw轴左16 cm、LiDAR同轴”是错误的，以本说明为准。

随后用户要求采用此前约15.3 cm的录包拟合结果。当前车辆入口实际采用202408旋转段的完整水平拟合向量 `[0.015293518836158204, 0.15267217555738066]` m，即前1.53 cm、左15.27 cm，水平半径15.34 cm。用户给出的左16 cm保留为安装关系描述，运行参数采用上述拟合值，不把半径误当成仅y分量。

## 当前采用的关系

- 雷达正装，随yaw转动。
- yaw轴→LiDAR：水平前后0、向左0.16 m，依据用户安装描述；不是用录包拟合值替代测量。
- 底盘中心→yaw轴：本次没有独立确认，配置保留unknown，不写成左16 cm，也不擅自写成0。
- 以前的底盘顶面20 cm、雷达再高5 cm保留为名义高度描述，不升级为现场精密标定。

控制参考点选在**yaw轴上、与LiDAR测量中心同一高度的点**，该点到LiDAR的运行平移为 `[0.0152935,0.1526722,0]` m。这里z=0是参考点的高度定义，不是认定雷达与云台底座等高。雷达正装、轴向平行为当前用户安装约定。

实际计算：

```text
LiDAR世界位置 = IMU世界位置 + R_IMU * 工厂IMU→LiDAR
yaw轴参考位置 = LiDAR世界位置 - R_IMU * [0.0152935, 0.1526722, 0]
nav_reference = yaw轴参考位置的x/y投影，导航平面z=0
```

因此底盘固定、云台旋转时，雷达可以真实绕圈，而导航参考应接近不动。原始LIO与去畸变点云仍保留真实传感器运动；不把地图一起反向挪动，不替换LIO yaw，不需要把 `contact_angle`重复加到速度转换中。

## 配置入口与兼容性

用户安装数据唯一入口：`src/rm_nav_bringup/config/vehicle_164_geometry.yaml`。新增车辆启动入口：

```bash
ros2 launch rm_nav_bringup vehicle_164_sensor_reference_state.launch.py \
  lio_params_file:=实际small_point_lio配置路径
```

车辆入口读取上述YAML，启用 `gimbal_axis` 模式及新ID `vehicle164_yaw_axis_fit153cm_debug_v3`。通用 `sensor_reference_state.launch.py` 默认依然是LiDAR中心，以保持旧录包及其他车辆兼容；它不会悄悄修改已有地图的参考点。

独立车辆组件脚本的state入口已改为车辆专用入口；capture/online统一使用新ID，关键帧目录切换到 `keyframes_fit153cm_debug_v3`，避免继续追加旧会话。这些是待部署的源码修改，没有远程修改原车程序。

完整底盘TF草稿仍为draft：删除错误的C→G平移，保留为null；G→IMU按上述拟合值减去工厂IMU→LiDAR生成名义草稿，完整高度仍未标定。该草稿不会因为本次水平关系确认就变成正式CalibrationBundle。

车辆入口保持 `reference_geometry_verified=false`，通过 `reference_fit_debug_approved=true`显式启用用户批准的录包拟合，且要求ID带debug标识。拟合不能同时被标成实测verified；状态话题会公布这两个标志。根几何文件的 `field_precision_verified=false` 和完整底盘Bundle的 `field_calibration_verified=false` 保持不变，没有声称已完成毫米级实测或完整底盘标定。

## 避障包络

旧0.54 m圆包络的推导曾把16 cm当成C→G偏移，该安装依据不再成立。本轮**没有擅自缩小包络，也没有将旧半径标成已验证**。数学包络函数不再隐含默认16 cm，测试中的16 cm明确是合成示例。

要精确确认绕新轴心的车体扫掠范围，仍需知道yaw轴相对于50×50 cm车体中心的位置。该未知量不妨碍对雷达进行轴心位置转换，但影响车体避障包络和完整底盘位置恢复。不能用已确认G→L替代这个独立参数。

## 验证范围

新增几何合同回归，防止16 cm再次误写回C→G，检查完整底盘草稿保持不可发布，检查拟合调试模式不能冒充实测标定。车辆专用入口使用实际MCAP回放验证，参考结果独立计算为录制LiDAR位置减去旋转后的拟合向量；不要求它与旧LiDAR中心轨迹相同。

ASUS当前不在线，本轮未部署、未发送运动。构建、测试与实际MCAP回放结果随本次交付保存；真实机械精度和车体包络尚需现场确认。

实际结果：

- 本轮WSL重建 `rm_nav_frames`、`rm_nav_bringup` 通过；上轮全14包构建通过。重跑受影响包测试，当前工作区汇总62项、0错误、0失败、1跳过。
- 数学单测12项、车辆几何合同2项通过；包含拟合配置显式批准、debug ID要求、不伪装实测verified。
- 使用新车辆入口回放真实202408 MCAP，1173个参考样本匹配1171个；轴心输出与独立计算值的最大XY差约5.6e-17 m，启动阶段未匹配的2帧未补造。
- 轴心参考最大水平变化约1.57 cm，末尾距首个匹配点约0.45 cm。这里使用拟合来源数据，验证的是配置生效和模型拟合效果，不能当独立实车精度证明；未重新运行LIO前端。
- 证据：[构建](evidence/vehicle164_fit153cm_20261022/fit153_build.log)、[测试](evidence/vehicle164_fit153cm_20261022/fit153_test.log)、[汇总](evidence/vehicle164_fit153cm_20261022/fit153_test_summary.log)、[实际录包回放](evidence/vehicle164_fit153cm_20261022/replay_result.json)。

提交前基线为 `f79700b`；本轮中文提交推送 `audit-p0-tf-costmap`，不合并main。
