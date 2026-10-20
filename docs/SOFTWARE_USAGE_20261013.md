# 最新软件使用与架构说明

适用：Ubuntu 24.04 / ROS 2 Jazzy；本机 Windows 通过 WSL 构建。最新开发分支为
`audit-p0-tf-costmap`，不自动合并 main。权威本地 Git 目录为 `.dev/Adam_nav_2027`；
`rm_nav_v2` 是旧的非 Git 副本。车上原有工作区和进程未修改。

## 1. 架构与控制来源

```text
MID360 driver → raw cloud guard → small_point_lio（真实 IMU 6DoF 位姿与去畸变点云）
                                     ↓
真实安装 TF → sensor_pose_resolver → robot_localization（完整 chassis 6DoF）
                                     ↓
                       base_footprint_projection + chassis_state_bridge
                                     ↓
              /odom + 带来源的 ObservationBatch → odom_submap / keyframe archive
                                     ↓
关键帧 → small_gicp 相邻配准 → KISS/GICP 回环 → GTSAM → mapping_odom draft
                                     ↓
官方坐标对齐 + 有依据的自由空间掩码 → published MapBundle
                                     ↓
map_server + frozen_map_matcher → MapOdomManager（唯一 map→odom 发布者）
                                     ↓
Nav2 global planner → TDT Minimum Jerk → Omni 轨迹跟踪 → smoother → collision monitor
                                     ↓
                         motion_gate（真实健康与任务许可）
                                     ↓
                Omni PID + YawManager → 选择一个传输后端 → my_serial_py
```

MPPI Omni 仍保留为稳定基线与对照，`--controller-profile tdt` 才使用真正 TDT QP 后端。
TDT 不是旧的端点插值：复用固定版本上游 MinimumSnapOsqp/OSQP，使用实际优化时间与多项式，
连续时间速度/加速度/jerk 验证、重求解、polygon 碰撞和版本绑定。

TF：`map→odom→base_footprint→chassis`。EKF 不发布 TF，投影器唯一发布后两条动态变换，
保留 chassis 的 z/roll/pitch。`base_link` 静态别名指向 chassis。
动态云台：`chassis→big_gimbal_yaw→imu→lidar`，必须有真实时间戳编码器。
固定雷达：`chassis→imu→lidar`，不创建云台 TF 或虚拟编码器健康。

## 2. 构建、环境和软件验收

每个终端加载对应独立工作区，不能混用原车 overlay。第一次按 tools/bootstrap_*.sh 安装固定版本依赖；
TDT 需要 `bootstrap_tdt.sh`，前端需要 `bootstrap_small_point_lio.sh`，不是原版 point_lio。
ROS 依赖包括 Nav2、robot_localization、pointcloud_to_laserscan、foxglove_bridge、
ros2_control 和 ros2_controllers。全部依赖的 pin 见 tools 下 lock/config 文件和各 bootstrap。

本机已安装的路径与完整构建示例：

```bash
cd /mnt/c/Users/Admin/Desktop/新导航框架/.dev/Adam_nav_2027
export RM_NAV_DEPS_ROOT=/root/adam_nav_deps
export RM_NAV_BUILD_ROOT=/root/adam_nav_complete
export MAKEFLAGS=-j2
source tools/workspace_env.bash
mkdir -p "$RM_NAV_BUILD_ROOT"
cd "$RM_NAV_BUILD_ROOT"
colcon build --base-paths /mnt/c/Users/Admin/Desktop/新导航框架/.dev/Adam_nav_2027/src \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
colcon test --base-paths /mnt/c/Users/Admin/Desktop/新导航框架/.dev/Adam_nav_2027/src \
  --executor sequential --return-code-on-test-failure
colcon test-result --verbose
cd /mnt/c/Users/Admin/Desktop/新导航框架/.dev/Adam_nav_2027
bash tools/run_software_acceptance.bash
```

验收脚本仅启动 TEST_ONLY 替身、真实 ROS 组件和 PTY，不打开物理雷达/串口。
输出目录每次新建在 Linux `/tmp/adam_nav_acceptance.*`，不会覆盖前次报告。
包含 MPPI/TDT 全向理想底盘、定位/恢复、串口、固定安装、映射链测试。
`tools/local_build_test.bash offline` 仍是子集，不能替代完整验收。
原子地图/关键帧测试使用 Linux 文件系统；WSL `/mnt/c` 的 DrvFS 可能不支持所需 no-replace 原子重命名。

## 3. 先启动传感器和本地状态

固定雷达配置已经写入 `rm_nav_frames/config/vehicle_164_fixed_mid360.yaml`：
用户提供的 chassis→LiDAR 为 x=0、y=+0.08、z=+0.20 m、正装；内部 LiDAR→IMU 向量
采用固定上游 MID360 配置 `[-0.011,-0.02329,0.04412]`，其逆变换没有混用。
两条 TF 合成后必须得到用户安装位置。配置状态为 `user_provided_factory_internal`，没有改成现场 verified。
这些数值以 chassis 原点为参考；若现场高度参考不同，需先改真实安装数据。

驱动使用设备实际 IP 的 JSON，点云外部安装变换必须为恒等，安装位移不能在驱动和 TF 重复应用。
先独立启动 livox_ros_driver2（PointCloud2 模式），再运行：

```bash
export ROS_DOMAIN_ID=97
source tools/workspace_env.bash
ros2 launch rm_nav_bringup local_state.launch.py \
  calibration_file:="$(ros2 pkg prefix rm_nav_frames)/share/rm_nav_frames/config/vehicle_164_fixed_mid360.yaml" \
  lio_params_file:="$RM_NAV_DEPS_ROOT/src/small_point_lio/config/mid360.yaml" \
  allow_user_mount:=true enable_mid360_guard:=true
```

检查 `/livox/lidar`、`/livox/imu`、`/sensors/front_mid360/cloud_healthy`、`/lio/sensor_odometry`、
`/state/chassis_healthy`、`/odom`、`/localization/odom_submap`。固定安装没有 `/state/gimbal_healthy` 是正常的。
有动态安装时改用真实 CalibrationBundle，不使用此固定安装配置。

## 4. 建图、对齐、发布

保持雷达/本地状态链运行，在新终端启动 recorder：

```bash
ros2 launch rm_nav_bringup mapping_capture.launch.py \
  archive_root:=/home/asus/nav_data/new_mapping \
  calibration_id:=vehicle164-user-mount-mid360-factory-20261013
```

结束后先停 recorder，再使用输出的 session 路径。单段最多 500 帧，超限必须换新 session，
不能宣称跨段全局拼接已经实现。保留原始 archive 与失败样本。

```bash
ros2 run rm_nav_mapping offline_graph_optimizer /绝对路径/session /绝对路径/optimized --enable-loops
python3 tools/create_map_bundle.py --optimized-dir /绝对路径/optimized \
  --output /绝对路径/optimized/draft.json
python3 tools/fit_official_alignment.py --correspondences /绝对路径/verified_survey.json \
  --output /绝对路径/alignment.yaml
python3 tools/publish_map_bundle.py --draft /绝对路径/optimized/draft.json \
  --alignment /绝对路径/alignment.yaml --output /绝对路径/published_map \
  --free-space-mask /绝对路径/official_mask.pgm \
  --free-space-metadata /绝对路径/official_mask.yaml
```

survey JSON 必须包含 `status: verified`、`source_frame: mapping_odom`、`target_frame: map`、
`official_source`、实测 `height_offset_m` 与至少三组非共线 `correspondences`，每组包含 `mapping_odom:[x,y]`、`map:[x,y]`。
只拟合刚体 SE2，不自动拟合比例；默认 RMSE≤3 cm、最大误差≤6 cm。输入坐标的官方来源由操作者确认，算法不能证明来源。

掩码为灰度 PGM：255 自由、0 占据、205 未知。metadata 包含 `status: verified`、`coordinate_frame: map`、
与 alignment 一致的 `official_source`、`resolution`（米）、`origin:[x,y,0]`（图像左下角）。
可由官方 CAD 栅格化后人工核对通行区；实际障碍物仍由 PCD 高度筛选覆盖到掩码中。
只有点云端点时不推断自由空间，生成的地图只可供定位，`navigation_ready=false`。
输出包含对齐 PCD、occupancy.pgm/map.yaml、terrain_reference.json、版本和来源。
terrain 当前是观测最小高度参考，`traversability_verified=false`，不是已验收坡度/可通行地图。
没有官方资料时到此保持 draft，不自动制造正式场地地图。

## 5. 定位与导航

独立启动正式导航，实测底盘 polygon 必须填写实际值，下面占位符不能直接运行：

```bash
ros2 launch rm_nav_bringup vehicle_navigation.launch.py \
  map_bundle:=/绝对路径/published_map/map_bundle.json \
  footprint:='实际测量的 [[x,y],...]' \
  control_backend:=direct_omni enable_physical_output:=false start_serial:=false
```

该入口加载 PCD、map_server、实际 matcher/MapOdomManager 和 TDT Nav2 profile。
不启动另一套 LIO，不创建静态 map→odom。场地原点较远时，在 Foxglove/RViz 发布带当前时间的
`/initialpose`（map 下 base_footprint 位姿）；两次一致真实配准前不会建立 map→odom 或健康。
已定位后的大跳变需走原有恢复事务，不能靠 `/initialpose` 绕过。

使用 `/nav/submit_goal`（`rm_nav_interfaces/srv/SubmitGoal`）提交 map 下 PoseStamped。
Supervisor 在预规划、定位与任务状态均通过后发布运动许可；直接发 NavigateToPose 不会自动获得其所有权。
查看 `/nav/task_status`、`/nav/motion_gate_reason` 与 `/localization/status_reason` 排查等待原因。

direct_omni 为原协议的直接适配，默认要求实测速度和 drive_ready。
用户暂不提供速度反馈时可以显式选择 `require_measured_feedback:=false readiness_topic:=/hardware/serial_healthy`，
此时只有速度前馈，仍要求真实 odom、串口报文、定位、Costmap、任务许可。
物理输出和串口开启必须显式设置 `enable_physical_output:=true start_serial:=true serial_port:=实际端口`。
本机验收未执行这些物理选项；现场还需核对 yaw 零点/方向、限速与下位机 watchdog。

`control_backend:=ros2_control` 使用独立 HardwareInterface + 两个标准 ForwardCommandController，
必须有真实 `/hardware/drive_feedback`（速度、yaw、ready、stopped、时间、递增序号），
其 state 不能从指令或 EKF 伪造。旧 RX26 没有这些字段，因此现有电控协议下该闭环后端不能自动激活。
硬件和控制器默认 inactive，需要完成实际反馈/停车验收后管理生命周期。
两种后端由同一启动器互斥选择；my_serial_py 是唯一串口 owner。

## 6. Foxglove

```bash
ros2 launch rm_nav_bringup foxglove_visualization.launch.py \
  preview_max_voxels:=50000 preview_voxel:=0.15 preview_publish_hz:=1.0
```

连接 `ws://实际车载IP:8765`，从布局菜单导入 `src/rm_nav_bringup/config/foxglove/mapping_low_latency.json`
或 `navigation_tdt.json`。格式依据 [Foxglove 官方布局示例](https://github.com/foxglove/tutorials/tree/main/integrations/ros1/transforms/foxglove_layouts)，
导入方法见 [官方文档](https://docs.foxglove.dev/docs/visualization/layouts)。本轮完成 JSON 和话题检查，桌面客户端实际导入尚未验证。

建图视图固定 frame=odom；导航视图=map。默认显示累积预览、轨迹、TF 与状态，导航另显示 map、costmap 和 TDT path。
原始点云 `/livox/lidar`、去畸变云、子图和完整 PCD 都可在 Topics 打开，但默认不同时订阅高带宽云。
TDT 还发布 `/nav/tdt/trajectory`、`/nav/tdt/environment`、`/nav/tdt/report`、
`/visualization/tdt_path`、`/visualization/tdt_costmap`，能区分实际轨迹、求解失败和环境版本。
TF 缺失先看上游健康/时间/固定 frame；不要新建静态 map→odom 来消除警告。

## 7. 尚需实机验证

代码、理想底盘及公开传感器回放不等于上车验收。车载主机目前不可用，未确认新雷达 IP、
当前串口 payload、yaw 正方向、硬件 watchdog、实测 footprint 和官方场地图。
这些输入和物理结果不能由开发代码补造。报告见 PUBLIC_MID360_ACCEPTANCE.md、NAVIGATION_INTEGRATION_ACCEPTANCE.md。
