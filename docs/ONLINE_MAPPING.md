# 在线回环建图使用与验收

在线后端在已提交的关键帧上做后台回环检测和历史地图更新，不修改 small_point_lio 的 odom。
复用 KISS-Matcher + small_gicp 双向配准、现有回环门限、GTSAM Pose3 图优化及优化后残差/局部形变/移除回环对照检查。
当前为**新关键帧触发的有界图优化**：接纳新回环后用已有 GTSAM LM 后端优化全图，不宣称已经实现 iSAM2。
相邻因子取实际记录的 LIO/状态 odometry 相对位姿；离线正式交付仍使用原有相邻 VGICP 精化链。

## 怎么运行

先启动本地状态、Foxglove 和关键帧 recorder，拿到其实际 `session_*` 目录。
在线后端可以在录制中途接入，会从第 0 帧补读，且不会跳过积压帧。
当前 ASUS 已构建独立 `install_online` overlay；它与既有 `install` 分开。

```bash
cd /home/asus/nav_2027_vehicle_config_164
bash start_online_mapping_20261020.bash /绝对路径/当前的session
```

通用 ROS 启动方式（每个新终端先加载相应构建环境）：

```bash
ros2 launch rm_nav_bringup online_mapping.launch.py \
  session:=/绝对路径/当前的session \
  calibration_id:=实际录制标定ID
```

当前实现匹配固定 MID360 的 `mid360_main/front_mid360/base_footprint` 归档契约，
不把其他安装来源悄悄当作同一参考点。切换 session 须停止并重启在线后端，旧录制文件保留。
只读取 recorder 原子提交的 keyframe_N 目录，不读取 `.partial`，也不向 recorder 的目录写入结果。
在线不反复计算原始文件哈希；共享归档读取器保留离线优化原有哈希检查行为。

## Foxglove 看什么

复用 `ws://192.168.1.164:8765`，不要再开第二个 bridge。
导入 [mapping_online.json](../src/rm_nav_bringup/config/foxglove/mapping_online.json)，或新建 3D 面板并设置
Fixed/Display frame=`mapping_odom`，启用下列三个显示层：

| 话题 | 意义 |
| --- | --- |
| `/mapping/optimized_map_preview` | 优化地图的低带宽预览，最多 5 万点，覆盖完整当前地图范围，适合 Wi-Fi |
| `/mapping/optimized_path` | 优化后的全部关键帧历史轨迹 |
| `/mapping/loop_graph` | 黄色关键帧、红色已接纳回环边 |
| `/mapping/optimized_map` | 完整有界体素地图，可选订阅；大地图时显著增加带宽 |
| `/mapping/online_status` | revision、keyframes、accepted_loops、candidates_tested、loops_rejected、pending_keyframes、耗时及失败原因 |
| `/mapping/mapping_odom_T_odom` | 最新关键帧处的全局修正 Pose，父参考 mapping_odom；这是诊断消息，不是 TF |

原有 `/visualization/live_map_preview` 仍为未做回环修正的 odom 累积预览，可放在另一个 odom 参考面板对照。
不要在同一面板把两个独立参考系未经变换直接叠加。本后端不发布 map→odom，也不把建图结果冒充正式场地 map。
若要看优化坐标下的机器人位置，optimized_path 的末端就是最新关键帧位置；当前不是高频完整机器人 TF 树。

接纳回环时，所有历史关键帧按新位姿重新构建地图，替换旧版本，不往旧地图追加一份修正点云。
地图、轨迹、回环图使用相同的末关键帧源时间戳。跨话题 DDS 到达不保证同一瞬间，比较时应核对时间戳。
没有新关键帧时地图不持续刷新；状态仍每秒更新。这不等于传感器断流。
状态 JSON 当前数值和布尔由 property_tree 序列化为字符串，数值分析须转换类型。

## 资源、失败和正式地图边界

- 单段最多 500 关键帧，每帧缓存最多 20000 个确定性采样点，原始点云继续留在磁盘。
- 在线地图默认 0.15 m 体素、最多 500000 体素，不淘汰历史区域。超限明确 fault，保留最后有效地图，不能声称无限规模建图。
- 每个新关键帧默认最多 2 个候选，至少相隔 10 帧/10 秒，候选半径 3 m；单帧最多接纳一条回环。
- 默认回环阶段 3 秒**软预算**。第三方配准调用不能中途抢占，超时结果丢弃，不作硬实时承诺。
- GTSAM 求解另受 500 节点/既有迭代数限制；耗时会报告。在线 worker 与 ROS 回调分开，部署进程 nice=10。
- 无效输入、关键帧缺号、来源混杂或资源超限会锁存 fault，保留上一有效快照与原始数据，需要处理原因后重启。
- `upstream_healthy` 反映当前车体健康心跳；它和 `fault`、积压量必须一起看，不伪造健康数据。
- 本阶段实时地图由采样关键帧构建，正式高保真交付仍可在停止录制后从原始 CDR 重新离线优化。
- 在线结果不自动发布正式 MapBundle，不绕过官方坐标对齐、占据图自由空间来源或定位验收。

## 可复现测试

```bash
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
python3 tools/smoke_online_mapping.py
python3 tools/smoke_loop_enabled_mapping.py
```

`smoke_online_mapping.py` 固定 Domain=197，仅使用 TEST_ONLY 人工闭环归档，
验证原子提交、真实 KISS/GICP/GTSAM、历史位姿修正、地图/路径时间一致、损坏输入拒绝与原始文件保留。
其成功不能证明真实场地回环精度、长时间稳定性或物理导航通过。
## ASUS 验收记录（2026-10-20）

代码检查基线为 feature `541413d`；开始时 HTTPS fetch 被重置，随后 SSH fetch 成功，未发现更新覆盖。
原运行链持续使用已有 install，本轮在独立 build_online/install_online 构建新后端，原车 Docker/工作区没有修改。

```bash
source /home/asus/nav_2027_vehicle_config_164/remote_env_20261020.bash
cd /home/asus/nav_2027_vehicle_config_164/releases/20261020
colcon build --build-base build_online --install-base install_online --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install_online/local_setup.bash
colcon test --build-base build_online --install-base install_online --executor sequential --return-code-on-test-failure
colcon test-result --test-result-base build_online --verbose
python3 tools/smoke_online_mapping.py
python3 tools/smoke_loop_enabled_mapping.py
```

- 全 14 包构建成功；最终全量回归 **50 tests、0 errors、0 failures、1 skipped**，跳过为可选 libscrc。
- C++ 测试实际调用 KISS/GICP/GTSAM，验证闭环修正历史、单平面拒绝、非法序号和容量失败保留原快照。
- ROS 测试使用默认 3 秒软预算，12 帧接受 2 条回环，历史位姿改变，地图/路径同时间戳，损坏第 13 帧锁存 fault。
- ROS 测试总墙钟 17.73 秒、CPU 122%、峰值 RSS 92516 KiB；这是整个 fixture/子进程树的统计，不是 500 帧实车峰值或 UI 延迟。
- 原有离线闭环回归通过，接受 2 条回环，证明共享 CDR 读取器没有破坏原离线路径。
- 当前真实 session 为 `session_1792512606735936939_1931153367`，已处理 29 帧、接纳 6 条回环、测试 19 候选、拒绝 13、积压 0、fault=false。
- 实测优化图/路径/地图均有输出；地图 10070 点，在线进程 PID=55306、nice=10，采样时 RSS 68908 KiB，累计 CPU 30.3%（含补读阶段）。PID 仅适用于本次存活实例。
- 20 秒探针接收原始云 101、LIO 云 130、odom 165 条，chassis/submap 最后均健康。接收计数不是驱动丢帧统计，不据此宣称精确吞吐或零延迟。
- Windows 经真实 Foxglove WebSocket 接收到 10070 点、mapping_odom 下的优化预览，传输检查通过；桌面布局导入和渲染未自动验证。

证据：[ROS fixture](evidence/online_mapping_fixture_20261020.json)、[真实链探针](evidence/online_mapping_live_20261020.json)、
[WebSocket 接收](evidence/online_mapping_foxglove_20261020.json)。
现场日志位于 `runtime_20261020/online_{build_final,test_full,ros_smoke_final,offline_regression}.log`；
当前后端日志在 `runtime_20261020/mapping_20261021_000953/online.log`，此目录名来自 ASUS 时钟。

首次编译发现 Jazzy rclcpp::Time 不提供 to_msg，改用其消息转换后通过；失败日志保留为 online_build.failed1.log。
跨机时钟差使传输后的源码时间可能早于构建产物，因此强制更新本次源码时间后重新编译并执行上述最终测试，未调整系统时钟。

**NOT VERIFIED**：真实运动与闭环真值、长时间/500 帧在线资源峰值、断电恢复后的部署自动启动、桌面实际渲染、正式场地坐标与导航精度。
29 帧里的 6 条边只是通过了当前质量门和图验收，不能宣称为已证明正确的场地回环。
没有发送车辆命令，没有改 EKF、LIO 前端、生产 TF authority 或 my_serial_py 协议。
最终提交 SHA 与 push 状态见本次 Git 提交/远端 feature 分支，main 不合并。
