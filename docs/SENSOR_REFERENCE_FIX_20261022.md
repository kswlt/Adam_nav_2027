# 2026-10-22 旋转参考链修复与回归

后续用户明确安装语义：16 cm是yaw轴→LiDAR的左偏心，此前本文关于C→G16 cm的表述错误，见 [安装更正与车辆专用入口](VEHICLE164_GEOMETRY_CORRECTION_20261022.md)。下文测试数据保留为修复当时快照；通用默认入口不变，车辆专用入口使用用户确认的偏心。

## 修复范围与状态

基线 `22b3f2c`，开发分支 `audit-p0-tf-costmap`。已修复朝向采样滞后和旧速度覆盖两个源码问题，增加显式实测轴心转换。ASUS当前不可用，全部构建和ROS测试在本机Ubuntu 24.04 WSL / ROS 2 Jazzy执行；没有部署ASUS或发送实车命令。

原有 `tools/smoke_local_site_navigation.py` 未提交修改保留，未纳入本轮提交或本轮新测试工作区。原始MCAP、串口错误记录和首轮失败日志均保留。

## 1. 朝向采样滞后

位置：`src/rm_nav_bringup/scripts/sensor_reference.py::tick`，原来使用最近一帧 `self.yaw` 做当前速度转换。LIO约10 Hz，速度转换50 Hz，旧代码没有补偿采样到执行时刻的朝向变化，并把输出重新标记为当前时刻。连续旋转时这会形成周期性方向滞后。

修复：`sensor_reference_math.py::HeadingTracker` 从连续两帧真实LIO求最短角yaw变化率，对当前输出时刻进行**有界恒角速度外推**。默认最多0.12 s、最多0.25 rad；超过时间、角度或物理角速度预算，或者不足两帧有效LIO，输出零平移。源TF和源位姿仍保留原始时间戳，不外推地图位姿。

该模块不是新EKF，不修改small_point_lio，也没有把编码器角当世界yaw。恒角速度是短时模型；突然加减速时仍可能有误差，0.25 rad是外推量上限，不是对真实方向误差的保证。

`/nav/sensor_command_status` 新增：

- `heading_source_stamp_ns`：实际LIO源时间。
- `heading_prediction_age_s`、`heading_prediction_correction_rad`：本次外推时长和角量。
- `heading_status`：等待、正常外推或拒绝原因。
- `velocity_point`、`calibration_id`：当前控制参考点与几何版本。

真实数据按录制到达时间模拟50 Hz转换，比较参考为后续LIO的朝向插值，**不是独立真值**：

| 数据 | 原持有朝向误差P95 | 修复后P95 | 原最大值 / 修复后最大值 | 超预算/未就绪tick |
|---|---:|---:|---:|---:|
| 202129 | 0.024° | 0.041° | 0.053° / 0.094° | 98/5315 |
| 202408 | 0.040° | 0.067° | 15.354° / 5.178° | 140/5980 |
| 202751 | 5.804° | 0.170° | 7.337° / 5.493° | 123/5949 |
| 203205 | 3.843° | 1.292° | 16.782° / 7.025° | 84/5350 |

两种误差在同一组被接受tick上比较；拒绝tick代表零输出区间，未当成零误差。静止时外推会略放大LIO角度噪声，不能声称每一帧都改善。实车可用率、加速度阶段和下游串口延迟仍需要验证。

## 2. 乱序速度覆盖

位置：`sensor_reference.py::velocity`。旧实现只检查消息年龄，未检查速度源时间是否比已接受的指令更新，因此一条仍在超时窗口内的旧速度可以覆盖新速度。

修复：仅接受严格递增且格式合法的源时间戳；重复、倒序、未来或过期指令清除当前平移命令。恢复权限不会恢复缓存的旧速度。通过真实ROS进程发送仍新鲜但比上一条旧1ns的指令，验证输出归零；之后的新指令可恢复。

## 3. 控制参考点不再隐含等同于LiDAR中心

旧实现唯一几何为工厂IMU→LiDAR平移；代码/状态却曾称其为“lidar_yaw_axis”，会把LiDAR中心与机械yaw轴混淆。

新增两种显式模式，均输出原有 `nav_reference` 和 `sensor_yaw_reference`，保持消息和串口协议兼容：

| 参数 | 默认值/要求 |
|---|---|
| `reference_point` | `lidar_center`；可选 `gimbal_axis` |
| `gimbal_axis_to_lidar` | `[0.0,0.0,0.0]`，在传感器轴中表示轴心→LiDAR的**实测**平移，单位m |
| `reference_geometry_verified` | 默认false；轴心模式必须已完成实际几何确认 |
| `calibration_id` | 默认 `vehicle164_sensor_axis_factory_v1`；轴心模式必须使用不同的新ID |
| `heading_max_extrapolation` | 0.12 s，允许配置范围(0,0.25] |
| `heading_max_correction` | 0.25 rad，允许配置范围(0,0.5] |

轴心模式按 `p_G=p_I+R_I*(t_I_L-t_G_L)` 进行完整三维参考点转换，再投影到导航平面；协方差使用同一参考点杠杆臂传播。工厂IMU→LiDAR静态TF不变，原始/去畸变点云不被平移或重标时间。

如果选择默认LiDAR中心却填非零轴心偏移，启动拒绝，避免参数静默不生效。轴心模式缺少实测确认或仍使用旧标定ID，也拒绝启动。**15.3 cm录包拟合未被写入默认配置。** C→G左16 cm不是这里的G→L，不得直接照搬。

启动参数通过 `sensor_reference_state.launch.py`传递，观测适配器及子图使用同一个 `calibration_id`。参数切换必须同时重启状态链、开启新关键帧会话并使用新ID录制；不得继续往旧会话追加或把旧MapBundle悄悄当作新标定地图。现有录包默认模式重放结果保持一致。

轴心几何测量后，仍需复核车体包络、局部Costmap参考点和实际电控xy参考点。新模式不会自动恢复真实底盘朝向、轮速或停车反馈；当前50×50 cm车体和C→G偏移也不意味着未知G→L下的旧包络已经验证。

## 编译、测试和实际结果

独立WSL目录：`/root/adam_nav_bugfix_20261022`；第三方依赖沿用 `/root/adam_nav_deps`。不是车载ASUS。

```bash
cd /root/adam_nav_bugfix_20261022
export RM_NAV_DEPS_ROOT=/root/adam_nav_deps
source tools/workspace_env.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

- 全量14个包构建通过，约2分4秒；包含此前未在ASUS完成构建的 `ContactAngleStamped` 接口。
- 全量61项测试：0 errors、0 failures、1 skipped。构建存在未修改的上游TDT编译器警告和ros2_control弃用警告。
- 新增/扩展：朝向跨±π、外推超限、逆序源时间、实测模式启动契约、旋转轴心位置保持、乱序速度拒绝。
- 10 Hz LIO / 1.2 rad/s恒速旋转 / 50 Hz输出ROS回归通过；这是TEST_ONLY理想数据，不能将近零模型误差当作实车精度。
- 原Adam全链路PTY测试通过：54-byte TX / 26-byte RX、单次符号转换、业务yaw透传、约50Hz发送、质量/权限/串口/定位断流归零，未修改电控协议。
- 初轮测试曾有缺失测试文件和夹具订阅积压/转向未稳定问题；新独立工作区补齐测试文件，夹具及时处理回调并在转向稳定后仍按原精度检查。初轮失败日志保留，不作为功能通过依据。

真实MCAP状态链回归命令：

```bash
python3 tools/replay_sensor_state_mcap.py \
  --bag /mnt/c/Users/Admin/Desktop/新导航框架/实车原始数据备份/20261021_云台旋转与闭环/run_20261021_202751 \
  --output log/real_mcap_state_replay
```

独立Domain209、`use_sim_time`、`/clock`；只回放原始点云/IMU和录制LIO/去畸变点云，排除旧TF、旧健康和旧参考输出。真实点云重新通过质量门，1180个参考样本中匹配1170个，匹配样本XY差为0。启动发现阶段未匹配的10帧不伪造补齐。没有物理速度发布者。这验证默认状态适配器兼容性，**不是重新运行前端LIO或完整Nav2导航验收**。

证据：[全量构建日志](evidence/sensor_reference_fix_20261022/build_result.log)、[完整测试日志](evidence/sensor_reference_fix_20261022/test_result.log)、[测试汇总](evidence/sensor_reference_fix_20261022/test_summary.log)、[首轮失败记录](evidence/sensor_reference_fix_20261022/initial_partial_workspace_test.log)、[真实MCAP结果](evidence/sensor_reference_fix_20261022/real_mcap_result.json)、[原串口PTY结果](evidence/sensor_reference_fix_20261022/legacy_transport_result.json)、[四组朝向重放指标](evidence/gimbal_offline_20261022/heading_fix_replay.json)。

四组朝向回归在Windows Python执行：

```powershell
python tools/replay_sensor_heading.py --raw-root "C:/Users/Admin/Desktop/新导航框架/实车原始数据备份/20261021_云台旋转与闭环" --output docs/evidence/gimbal_offline_20261022/heading_fix_replay.json
```

## 尚未关闭的问题

约15.3 cm偏心的机械归属和当前固件角度语义尚未实物确认，因此未切换实车轴心模式。第三组79 cm末尾距起点没有路线真值，不能靠这轮代码宣称解决。ASUS部署、实车方向/跟踪、最终地图定位、完整Nav2运行及Foxglove端到端延迟仍NOT TESTED。本轮源码问题已修复并完成上述软件验收，整车“彻底修好”仍需这些现场证据。

中文提交推送至 `audit-p0-tf-costmap`，不合并main；实际SHA及远端状态见本次交付和git历史。
