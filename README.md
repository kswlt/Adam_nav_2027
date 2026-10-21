# Adam_nav_2027 使用说明

> **当前车载运行代码在 `audit-p0-tf-costmap` 分支。main 此次同步使用文档，尚未合并该分支源码。** 下方文件链接指向运行分支；拉取代码请明确选择：

```bash
git clone --branch audit-p0-tf-costmap https://github.com/kswlt/Adam_nav_2027.git
```

**最新地图检查：当前会话LIO发生公里级位置跳变，在线后端已锁存故障，离线优化也拒绝。已停止关键帧录制并保留原档案；导出的是故障前1帧、2806点的检查包，不是可导航地图。** 见 [本次导出结果和下一步](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/MAP_EXPORT_20261021.md)。下文启动快照不代表这次故障后的健康状态。

这是 RoboMaster 全向车导航工程。环境：Ubuntu 24.04、ROS 2 Jazzy、Nav2 Jazzy；雷达前端是 **small_point_lio**，串口沿用 **my_serial_py**。

**现在默认开发的是“雷达参考导航模式”：上位机负责定位、建图、规划和平移；电控负责底盘自旋、轮子控制和大 yaw。** 不再要求用 `contact_angle` 推算底盘朝向。原有完整底盘模式仍保留，两种模式不能同时启动。

**电控源码核对（10月21日）：** 电控内部读取大yaw编码器转换xy，上传的`contact_angle`却写死为0，所以不回传角度也能执行这条速度链。串口“yaw”在当前电控特定模式下实际当作**度/秒速度**，不能把LIO yaw填进去。导航无自瞄跟踪时大yaw默认80°/s；零xy/零yaw不等于整车停止。还发现断流缓存未清零及导航底盘自旋被置零的源码问题；是否与车上固件一致待核对。详见 [电控yaw逐项解释与源码证据](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/ELECTRICAL_YAW_AUDIT_20261021.md)。

本机与ASUS均已通过14包构建、56项测试（0失败、1跳过）；本机新模式和原有TDT/MPPI导航回归通过。

2026-10-21 已在 ASUS 独立目录部署并通过同样的56项测试。雷达参考状态、Foxglove、新关键帧会话与在线建图已启动。**按用户要求，真实串口发送已开启：约50Hz发送原Adam的54字节报文；当前xy为0，yaw缓存值为0°。自动导航尚未开启。** 内核写入抓取2.013秒共100包；这不是下位机执行确认。实际运行状态以话题和下面的会话查询命令为准。现场目录与实测记录见 [ASUS部署记录](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/ASUS_SENSOR_REFERENCE_20261021.md)。

## 先弄清三个状态

| 状态 | 可以做什么 | 不能据此认为什么 |
| --- | --- | --- |
| 建图 | 看扫描、雷达轨迹、累积点云；录制关键帧；在线检测回环并更新历史地图 | 有点云不代表已有正式导航地图 |
| 导航影子运行 | 在正式地图上定位、规划，查看将要发送的平移速度 | 不向串口发车速，不会让车自动跑 |
| 真实串口发送 | 已接通电控串口，当前发送零平移报文 | 不代表非零速度方向、停车或自动导航已验收 |

## 1. 连接车载电脑

```bash
ssh -p 2233 asus@192.168.1.164
```

车上独立目录是 `/home/asus/nav_2027_vehicle_config_164`。本工程不覆盖原车工作区，不停止原车导航程序。

每个新的 SSH 终端都先执行：

```bash
export NAV_BASE=/home/asus/nav_2027_vehicle_config_164
source "$NAV_BASE/remote_env_20261020.bash"
export NAV_RELEASE="$NAV_BASE/releases/20261021_sensor_reference"
export NAV_RUNTIME="$NAV_BASE/runtime_20261021_sensor_reference"
source "$NAV_RELEASE/install/local_setup.bash"
```

ROS Domain 是 **97**。这些路径属于当前独立部署，不要混用旧 `.145` 主机或原车工作区的环境。

## 2. 启动建图：按顺序执行

当前已启动，可以直接连接Foxglove。以下后台启动命令会识别本会话已有实例，不会重复创建；先前台运行的实例需自行停止后再切换。

查询某个组件：`python3 "$NAV_RUNTIME/vehicle_session.py" status state`。日志和PID记录在 `$NAV_RUNTIME`。
只停止新会话、保留原车程序：

```bash
for component in physical bag online capture candidate foxglove state driver; do
  python3 "$NAV_RUNTIME/vehicle_session.py" stop "$component"
done
```

需要重启时再按下列顺序启动：

**第一步：雷达驱动**

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" start driver
```

驱动使用 `runtime_20261021_sensor_reference/mid360_identity.json`：车体安装位移不能再在驱动里重复应用。

**第二步：雷达参考状态链**

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" start state
```

这个入口启动质量门、small_point_lio、传感器 TF、虚拟平面参考点及滚动子图；不启动底盘 EKF、机械 yaw TF 或串口。它使用新运行目录中的 `small_point_lio_live.yaml`，保留旧目录配置。

**第三步：Foxglove**

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" start foxglove
```

如果 8765 已有本工程 Bridge，不要再开一次。电脑上的 Foxglove 连接：`ws://192.168.1.164:8765`。

**先确认真的有数据：**

```bash
ros2 topic hz /lio/deskewed_odom_cloud
ros2 topic echo /state/sensor_reference_healthy std_msgs/msg/Bool --once
ros2 topic echo /state/sensor_reference_reason std_msgs/msg/String --once
ros2 run tf2_ros tf2_echo odom front_mid360
```

`hz` 和 `tf2_echo` 会持续显示，观察后按 Ctrl+C。健康值应为 true；false 时看 reason，不能靠手工发布 true 绕过。

## 3. 保存关键帧，并开启在线回环

仅看累积预览不会自动留下完整建图档案。要保存，另开终端：

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" start capture
ros2 topic echo /mapping/recorder_status std_msgs/msg/String --once
```

当前部署已启动录制；组件日志和 `/mapping/recorder_status` 会给出 `session_...` 完整路径，新档案位于 `$NAV_RUNTIME/keyframes`。复制它，再开一个终端：

```bash
# 把下面路径换成 recorder_status 中的完整 session。
export NAV_SESSION=/实际完整路径/session_实际编号
python3 "$NAV_RUNTIME/vehicle_session.py" start online "$NAV_SESSION"
```

后端边读取关键帧，边做双向 KISS/GICP 回环验证和 GTSAM 优化；回环通过后，重建历史点云地图。默认最多 500 个关键帧，达到容量或遇到错误会报 fault，保留已有档案，不静默覆盖旧数据。

查看 `ros2 topic echo /mapping/online_status std_msgs/msg/String --once`：`keyframes` 是帧数，`accepted_loops` 是通过门限的回环数，`pending_keyframes` 是积压，`fault` 应为 false。没有回环不一定是故障，也可能还没回到旧区域。详情见 [在线建图指南](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/ONLINE_MAPPING.md)。

### 第一次实车采样：先遥控，不开自动输出

保持空旷环境，准备好后先启动限时3分钟的原始数据录制：

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" start bag
```

纯遥控采样时先执行 `python3 "$NAV_RUNTIME/vehicle_session.py" stop physical`，避免与串口命令叠加。再用遥控器低速前进约1米、返回起点，然后停车。观察雷达轴轨迹是否往返、地图是否重影；关键帧也会随平移保存。
结束后运行 `python3 "$NAV_RUNTIME/vehicle_session.py" stop bag`，MCAP保存在 `$NAV_RUNTIME/bags`；不手动停止时3分钟自动结束。任务完成不等于自动导航已通过，后续还要受控检查电控速度方向和停车。

## 4. Foxglove 中每项是什么意思

**看实时建图：**导入 [雷达参考建图布局](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/src/rm_nav_bringup/config/foxglove/sensor_reference_mapping.json)，3D 的 Fixed frame 和 Display frame 都选 `odom`。

**看回环优化地图：**导入 [在线优化布局](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/src/rm_nav_bringup/config/foxglove/mapping_online.json)，参考系选 `mapping_odom`。两套地图在独立坐标系，不要强行叠加后判断错位。

| 话题 / 坐标 | 显示的内容 | 怎么用 |
| --- | --- | --- |
| `/livox/lidar` | 雷达刚扫描到的原始点云 | 查雷达是否正常；不是整张地图 |
| `/lio/deskewed_odom_cloud` | small_point_lio 去畸变、放进 odom 的当前扫描 | 看实时配准；仍主要是当前扫描 |
| `/localization/odom_submap` | 近期观测组成的滚动局部子图 | 给定位匹配使用；不是永久全图 |
| `/visualization/live_map_preview` | 历史扫描累积预览 | 直观看建图扩大；不包含历史回环修正 |
| `/mapping/optimized_map_preview` | 经过后端修正的全历史地图预览 | 在线建图时优先看这个 |
| `/mapping/optimized_map` | 后端完整体素地图 | 点较多，日常显示用 preview |
| `/visualization/sensor_reference_path` | 雷达轴平面参考点走过的轨迹 | 不表示底盘中心的轨迹 |
| `/mapping/optimized_path` | 优化后的历史参考点轨迹 | 回环后旧轨迹可能一起调整 |
| `/mapping/loop_graph` | 关键帧和回环连线 | 看哪里发生回环，不是导航路线 |
| `/nav/tdt/trajectory`、`/visualization/tdt_path` | 规划得到的时序轨迹和显示路径 | 导航启动并接到任务后才有 |
| `/nav/sensor_cmd_vel_shadow` | 转换到当前雷达水平朝向的平移速度 | 只观察；不直接连接串口 |
| `nav_reference` | 位置跟随雷达轴，方向保持与 odom 对齐的虚拟导航坐标 | 不是车头，不表示底盘没旋转 |
| `sensor_yaw_reference` | 雷达朝向投影到水平面的坐标轴 | 影子速度的坐标系，不包含雷达倾斜 |
| `front_mid360` | 雷达真实朝向 | 雷达转动时它会跟着转 |
| `mapping_odom` | 后端优化地图的坐标 | 不等于正式比赛场地 map |

3D 点的红黄蓝通常是面板按高度或强度着色，**不是默认的可通行/危险标签**；在点云设置中查看 Color mode。

底部 `/visualization/map_cloud.header` 一直“等待消息”时，常见原因是没有加载冻结 PCD 地图。建图阶段改看上表的实时/优化点云即可。图表面板要手工选数值字段，例如影子速度的 `twist.linear.x/y`。

卡顿时：只打开一种大点云，优先 preview；关闭 full map 和原始点云同时显示，关闭长时间点云累积。预览本来会降频，不能拿它的刷新率当作 LIO 频率。

## 5. 怎么进入导航

先完成建图交付与正式场地对齐，得到 `status=published` 且 `navigation_ready=true` 的 MapBundle。draft、在线预览或随手保存的 PCD 不能直接代替它。

保持雷达参考状态链运行，另开终端：

```bash
ros2 launch rm_nav_bringup sensor_reference_navigation.launch.py \
  map_bundle:=/实际正式地图目录/map_bundle.json
```

该入口加载地图、冻结地图匹配、唯一的 MapOdomManager、Nav2 和只跟踪平移的 TDT 控制器。没有正式 Bundle 会直接拒绝启动。在 Foxglove 的 3D 面板用 Publish pose estimate 向 `/initialpose` 发布地图初值；位置选 **雷达轴参考点**。箭头是虚拟 nav_reference 在 map 中的方向，不是雷达当前扫描方向。初值只是匹配提示，匹配成功前不会获得运动许可。

定位健康后提交任务，下面的 x/y 必须替换为实际地图中的可行目标：

```bash
ros2 service call /nav/submit_goal rm_nav_interfaces/srv/SubmitGoal \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
ros2 topic echo /nav/task_status --once
```

此模式只要求到达位置，目标 orientation 不要求车辆转向。直接点击 `/move_base_simple/goal` 或直接发 Nav2 action 不会自动获得任务监管器许可，调试时优先使用上面的服务。

障碍物投影高度默认在 odom z=0 的 -0.10～1.50 m 范围；地面与车身回波过滤需现场检查，不能把有点云直接当作避障验收。

导航状态和速度依次看：`/localization/healthy` → `/nav/motion_allowed` → `/nav/cmd_vel_safe` → `/nav/sensor_cmd_vel_shadow`。任务接口见 [完整软件使用说明](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/SOFTWARE_USAGE_20261013.md)。停止本次任务或进程只会停止本工程平移输出；**不代表电控自旋也停止**。

当前影子模式不会打开串口。接物理输出前还要确认：电控接收 xy 的坐标方向、速度作用点是底盘中心还是雷达轴、偏心自旋速度补偿，以及下位机停止/超时行为。这些不能从 `contact_angle=0` 推断出来。

### 电控怎么接：保留原 Adam 协议

目标是下位机不改协议：仍使用原 `my_serial_py` 的115200、50Hz、54字节下发/26字节回传；xy按原方向转换并在打包时各取负一次。yaw目标、姿态、区域、对齐标志和预留位都保留。

新的兼容入口默认不打开串口，可先看候选命令：

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" start candidate
ros2 topic echo /hardware/legacy_adam/adapter_status --once
```

它不占用全局 `/cmd_vel`，避免旧业务节点的速度混入。传输采用旧上位机缓存yaw逻辑；不发布新的yaw目标，也不把LIO yaw填入电控字段。上位机仍叫yaw_degrees，但新提供电控在特定模式下将该数值用作deg/s；字段名不能代表固件执行含义。

### 真实发送的启动、查看和停止

当前ASUS已开启真实发送，串口为 `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`（ttyUSB0）。重启后需要真实发送时执行：

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" stop candidate
python3 "$NAV_RUNTIME/vehicle_session.py" start physical
python3 "$NAV_RUNTIME/vehicle_session.py" status physical
ros2 topic echo /hardware/legacy_adam/serial_healthy std_msgs/msg/Bool --once
ros2 topic echo /hardware/legacy_adam/adapter_status std_msgs/msg/String --once
```

关闭本工程的串口发送并释放串口：

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" stop physical
```

physical组件同时启动适配器和真实my_serial_py；不要再并行启动candidate或第二个串口节点。新速度走私有 `/hardware/legacy_adam/cmd_vel`，全局 `/cmd_vel` 没有发布者不等于串口没发。

当前没有正式地图定位与任务运动许可，适配器保持xy=0；不是让车自行驶向目标。yaw字段沿用原版缓存策略，初值数值0；当前电控导航默认扫描可能绕过这个值，详见上述源码审计。遥控接管尚未实测确认。停止串口也不等于电控自旋停车，当前固件源码未见导航断流缓存清零门控。

实测 [100个真实写入报文](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/evidence/asus_serial_tx_20261021.json)。非零速度方向、速度作用点、偏心自旋补偿、停车行为仍需现场验证。连接方式和协议对照见 [Adam兼容说明](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/LEGACY_ADAM_REPLACEMENT.md)。

## 6. 架构和安装几何

```text
MID-360 → 原始点云质量门 → small_point_lio
                              ├─ 雷达 6DoF 位姿 → sensor_reference → 虚拟平面参考与健康状态
                              └─ 去畸变点云 → 观测子图 → 关键帧 → 回环验证/GTSAM → 历史地图

正式 MapBundle + 观测子图 → 冻结地图匹配 → MapOdomManager → map→odom
正式地图 + nav_reference + 障碍物 → Nav2 → TDT 平移轨迹 → 碰撞检查/健康门
                                                   → 雷达系 xy 影子速度
电控（独立职责）：底盘自旋 / 大 yaw / 轮子控制 / 最终速度执行
```

TF 为 `map → odom → front_mid360_imu → front_mid360`，另有 `odom → nav_reference` 和 `odom → sensor_yaw_reference`。`map → odom` 只由真实地图匹配后的 MapOdomManager 发布。没有机械角反馈时，不发布冒充真实的 chassis/base_link 旋转。

用户确认：车外缘约 **50×50 cm**；yaw 轴在底盘中心左侧 **16 cm**；雷达与轴同轴，离地约 **25 cm**。导航以该轴为参考，使用半径 **0.54 m** 的保守圆包络覆盖旋转底盘。它比真实轮廓保守，窄通道通过能力会降低。工厂 IMU/LiDAR 内部外参不等于车体现场标定。

要做精确车体姿态、按实际矩形挤窄道，仍需真实机械 yaw 和车体状态；那是保留的完整底盘模式，见 [架构契约](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/architecture_contract.md)。

## 7. 开发、测试和交接

本次本机工作区：WSL `/root/adam_nav_sensor_reference`；依赖在 `/root/adam_nav_deps`。测试不连接物理串口。

```bash
cd /root/adam_nav_sensor_reference
source /opt/ros/jazzy/setup.bash
source /root/adam_nav_deps/livox_ws/install/local_setup.bash
source /root/adam_nav_deps/install_lio/local_setup.bash
export CMAKE_PREFIX_PATH=/root/adam_nav_deps/install:${CMAKE_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/root/adam_nav_deps/install/lib:${LD_LIBRARY_PATH:-}
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
```

依赖安装和其他机器的配置见 [源码构建指南](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/tools/deployment/20261020/README.md)。最新模式说明与测试记录见 [传感器参考模式](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/SENSOR_REFERENCE_MODE.md)；接续开发先读 [HANDOFF](https://github.com/kswlt/Adam_nav_2027/blob/audit-p0-tf-costmap/docs/HANDOFF.md)。硬件、比赛场地精度和真实停车仍需单独验证。
