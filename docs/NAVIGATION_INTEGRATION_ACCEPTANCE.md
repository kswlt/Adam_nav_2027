# TDT / 正式地图 / 控制集成验收

2026-10-13 本机 WSL Ubuntu 24.04、ROS 2 Jazzy，原生依赖和全 14 包构建通过。
`colcon test` / `colcon test-result --verbose`：49 tests、0 errors、0 failures、1 skipped
（原版 libscrc 可选兼容依赖未安装，原始 fixture 未改写）。

## 已实现

- Nav2 `rm_nav_control::TdtOmniController` 插件，实际生命周期、tf2、Costmap2DROS、Path 输入。
  从实际速度与 footprint 生成 TDT 时序轨迹；每轮检查最新 Costmap、原始点云质量心跳和 MapBundle 版本。
- 真实 TF 转换后的局部计划、有限前视、重规划、速度过滤；错误/取消清除轨迹。
- `/nav/tdt/{trajectory,environment,report}`、`/visualization/tdt_{path,costmap}` 可视化。
- 正式 Bundle 校验与 PCD 实际加载、map_server、占据栅格和 TDT 启动入口。
  端点云不能证明自由空间；只有官方来源一致的 verified mask 才允许 navigation_ready。
- `/initialpose` 通过 exact-time odom→base_footprint 转成临时定位提示；没有静态 map→odom，
  两次独立且一致的真实 GICP 配准前不建立定位。已经定位后的大变化仍需恢复事务。
- `vehicle_navigation.launch.py` 互斥选择 direct_omni / ros2_control，默认关闭物理输出与串口。
- ros2_control 使用实际 Jazzy controller_manager、两个标准 ForwardCommandController，
  HardwareInterface 真实反馈接口、独立命令租约、超时锁定；初始 inactive。
  不把逻辑 chassis 命令接口假称轮关节，不发布虚拟轮 TF。

## 软件证据

- 原生 TDT plugin 测试通过实际 tf2/costmap 检查坐标变换、障碍物、速度限制、取消和传感器失效。
- Nav2 理想底盘横移 1 m 目标，TDT action SUCCEEDED，末位置约 `[-0.04074,0.86602,0]`，
  最大横向命令约 0.15315 m/s；使用原有 Nav2 到点容差，不把容差内停点当作真值精度。
  障碍、scan 断流、命令超时、定位失效/心跳失效、权限关闭均停止。
- MPPI 对照同轮通过，保留独立报告。
- `smoke_initial_pose.py`：map 原点偏移 `[20,-10]`，提示不产生 TF，两个真实 GICP 结果后建立正确变换。
- `smoke_ros2_control.py`：实际 manager 和标准控制器加载为 inactive，无物理输出/虚拟 TF。
- `ros2_control_input_test`：实际 ROS 消息，度/弧度转换、反馈前置、独立 lease 超时、停止覆盖迟到候选。
- 固定/动态安装、定位恢复事务、取消失败、清图超时、任务恢复、mapping capture、PTY 集成均通过。
- `small_point_lio --with-state --with-mapping`：实际前端→resolver→EKF→Observation→GICP→归档通过，
  47 组 raw pose/cloud 与 1 个原始关键帧；断流不重放缓存。
- MapBundle 发布 9 项、启动契约 3 项、官方对齐 4 项测试通过。

本轮集成证据 `/tmp/adam_nav_acceptance.v7sBlC`；首次 runner 缺 `--with-state` 的调用失败保留，
修正调用后余下用例成功，见 `lio_mapping_afterfix.log`。修正后的完整入口为
`bash tools/run_software_acceptance.bash`。

尚未验证：实车运动、真实 DriveFeedback、STM32 watchdog/停车、真实 official MapBundle、官方坐标、
场地全尺寸 footprint/窄道/动态障碍、桌面 Foxglove 布局导入。
用户暂缓轮速反馈时 direct_omni 可显式选择纯前馈；不能把它说成闭环速度控制验收。
新风险包括严格地图/轨迹门限拒绝合法但未经测量的输入，需调查实际数据而非自动放宽。
