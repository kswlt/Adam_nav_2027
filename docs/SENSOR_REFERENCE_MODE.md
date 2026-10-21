# 雷达参考导航模式

2026-10-21 追加：独立的 [legacy_adam_transport](LEGACY_ADAM_REPLACEMENT.md) 可将影子xy接入原协议；默认仍关闭设备，不改此状态链的yaw/TF语义。

本模式适用于全向底盘自旋、大 yaw 独立扫描、上位机暂时拿不到底盘实际朝向的分工。
它不使用 `contact_angle`，也不将 LIO yaw 解释成云台相对底盘角度。部署暂缓，按用户要求先在本机 WSL 验证。

## 坐标和控制合同

| 名称 | 含义 | 发布者 |
| --- | --- | --- |
| `odom → front_mid360_imu` | small_point_lio 的真实 IMU 6DoF 位姿，原始时间戳 | sensor_reference |
| `front_mid360_imu → front_mid360` | 工厂内部平移 `[-.011,-.02329,.04412]` m、单位旋转 | sensor_reference 的 static broadcaster |
| `odom → nav_reference` | 雷达轴位置的 x/y 投影；z=0、朝向与 odom 一致 | sensor_reference |
| `odom → sensor_yaw_reference` | 同一平面点，朝向为雷达朝向的水平投影 | sensor_reference |
| `map → odom` | 真实地图匹配结果；建图阶段不存在 | MapOdomManager |

`/sensor_reference/odom` 是虚拟参考的导航状态，x/y 协方差由 IMU 位姿协方差通过实际选用的参考点杠杆臂传播。默认仍为LiDAR中心；2026-10-22新增显式实测 `gimbal_axis` 模式及有界朝向外推，参数、标定ID切换和验证限制见 [旋转参考链修复](SENSOR_REFERENCE_FIX_20261022.md)。不要把上表默认LiDAR位置等同于未经测量的机械yaw轴。
固定的 z/roll/pitch/yaw 是该坐标的定义，不是真实底盘测量。速度来自参考点位置差分，保留未知速度协方差；不能作为轮速或停车反馈使用。
无真实机械角时，不发布 `chassis`、`base_footprint`、`base_link` 的假旋转，不将参考里程计投入原有 chassis EKF。

平移转换与原 Adam 的 `fake_vel_transform` 方向一致：

```text
vx_sensor = cos(yaw_sensor) * vx_reference + sin(yaw_sensor) * vy_reference
vy_sensor = -sin(yaw_sensor) * vx_reference + cos(yaw_sensor) * vy_reference
```

输出 `/nav/sensor_cmd_vel_shadow` 为 TwistStamped，frame=`sensor_yaw_reference`，角速度恒为0。
它不会发布 `/cmd_vel` 或 `/cmd_yaw_angle`。现有 my_serial_py 字节格式、负号约定和现有完整底盘适配器均未改动。

**速度的作用点与速度的坐标轴不同。** 当前跟踪的是雷达/yaw轴点；该点距底盘中心左偏16cm。
如果电控接受的是底盘中心速度，自旋会产生 `ω × r` 的差异。是否由原电控完成补偿、速度轴是否确为 yaw 水平轴，仍待核对；不能在缺少真实角速度和机械关系时假装已补偿。
xy=0只表示平移请求归零，不证明底盘自旋、大 yaw 或轮子已停止。

## Nav2 与避障

独立入口 `sensor_reference_navigation.launch.py`：

- 复用既有正式 MapBundle 校验、冻结地图匹配、MapOdomManager、Nav2、TDT Minimum Jerk。
- 两个 Costmap、BT、Collision Monitor 和 motion_gate 使用 `nav_reference`。
- TDT 的 `translation_only=true` 将路径朝向和初始 yaw 速度固定为当前虚拟参考朝向，输出角速度0；原模式默认false。
- 目标只要求位置到达，忽略目标朝向。专用 BT 不发 Spin/BackUp 恢复动作，仅启用 NavigateToPose；默认多点导航树仍需要 Spin，不在本模式启用。
- 状态健康订阅映射到真实 `/state/sensor_reference_healthy`，没有发布虚假的 `/state/chassis_healthy`。
- 不启动 Omni/YawManager 的底盘朝向适配器或串口。恢复事务仍关闭，因为没有真实停车反馈。

车外轮廓50×50cm，yaw轴偏心16cm，最远角到该轴约0.4802m；加5cm余量后小于0.54m。
Costmap 使用外切32边形覆盖半径0.54m整圆，Collision Monitor 再扩大15%。因此不依赖未知底盘 yaw，也更难通过窄道。
高度投影默认截取 odom z=-0.10～1.50m；需现场检查地面、自身结构和低矮障碍，软件回归不替代这些验证。

## 建图兼容

新链使用 `vehicle164_sensor_axis_factory_v1` ID，只描述雷达轴参考和工厂内部外参，未宣称机械角已标定。
新录制指定 `body_frame=nav_reference`；在线后端显式接受这一参考，默认仍为 `base_footprint`，不放宽旧 session 的参考一致性检查。
使用新目录和新 session，不能修改旧固定安装档案的标定 ID 来混用。
回环优化仅更新独立 `mapping_odom` 下的历史地图，不回写前端 odom 或伪造正式 map TF。

## 本机测试

本机独立源码/构建：WSL `/root/adam_nav_sensor_reference`；不访问串口、不连接原车工作区。
编译与回归记录在该目录 `build_report.log`、`test_report.log`、`test_result.log`，最终结果见本文件末尾。

```bash
source /opt/ros/jazzy/setup.bash
source /root/adam_nav_deps/livox_ws/install/local_setup.bash
source /root/adam_nav_deps/install_lio/local_setup.bash
export CMAKE_PREFIX_PATH=/root/adam_nav_deps/install:${CMAKE_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/root/adam_nav_deps/install/lib:${LD_LIBRARY_PATH:-}
cd /root/adam_nav_sensor_reference
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
ROS_DOMAIN_ID=203 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 tools/smoke_mppi_baseline.py --controller-profile tdt --sensor-reference
python3 tools/smoke_online_mapping.py --body-frame nav_reference
```

TEST_ONLY fixture 使用独立 ROS Domain 和理想传感器/地图，不能连接车载 Domain。
需要额外验证：真实动态 MID360 数据、实际电控坐标与作用点、全局地图精度、低障碍与自身回波、网络负载、旋转和平移联动、下位机停止行为。

## 结果记录

基础 ROS fixture 已通过：90°坐标转换、LIO断流、原始质量失败、源时间过期、非法协方差、权限撤销、角速度拒绝、恢复不重放；物理速度/yaw发布者数量0。
- 检查基线：`d6fb61b88432d7342bdb4e904735504f84a765c8`；独立 feature 分支 `audit-p0-tf-costmap`，不合并 main。
- 本机全14包 `colcon build` 通过；`colcon test` / `colcon test-result --verbose`：53 tests、0 errors、0 failures、1 skipped（可选 libscrc）。
- 首次新 Nav2 fixture 复现 BT 激活失败：禁用 Spin 后默认 NavigateThroughPoses 树仍等待 spin action。修复为该 profile 仅启用 NavigateToPose，并使用无旋转恢复的专用树；旧模式不变。
- 修复后新 Nav2 profile 横移1m测试成功；目标姿态设为1rad，虚拟 yaw 始终0。生产默认40ms求解预算测试通过，不通过扩大生产预算制造通过结果。
- nav_reference 档案在线回归：12关键帧、2条接纳回环、22416地图点、历史位姿改变；坏第13帧拒绝且保留第12版地图，原始档案未改动。
- 已知构建警告：既有上游 TDT 源码编译器告警、ros2_control deprecated API；Nav2 Smac 对较大圆包络给出 inflation 性能建议。未通过关闭告警冒充无风险。
- 未验证：真实偏心自旋运动、物理串口输出、正式比赛地图和实际避障/停车；本轮没有采集新 MCAP，没有修改或删除既有原始数据。

修改范围：`rm_nav_bringup` 的新状态脚本、launch、专用 BT/布局与文档；`rm_nav_control` 仅增加默认关闭的 translation_only；`rm_nav_localization` 仅参数化 MapOdomManager 参考点；`rm_nav_mapping` 仅参数化档案参考坐标。算法复用 small_point_lio、tf2、现有 TDT/Nav2、small_gicp/KISS/GTSAM，没有新造 SLAM/EKF。

原有 TDT、MPPI 理想全向导航回归也通过，包含障碍停止、扫描断流、命令超时、定位健康/心跳丢失、任务许可撤销。
机器可读结果见 [sensor_reference_20261021.json](evidence/sensor_reference_20261021.json)。该文件保留首次 BT 失败的工具输出摘要和修复后结果；全部均是 TEST_ONLY 数据，没有把理想模型结果当作物理精度。

提交号与远端分支状态以包含本报告的 Git 提交及 HANDOFF 最新条目为准。
