# 时序轨迹跟踪软件验收（2026-10-13）

## 基线与范围

远端开发分支仍为 `55a2115b9ed1261b26b4eb4aeaee3282560617b2`，已通过 GitHub 连接器重新读取确认。
本机在前一轮轨迹/地图补丁之上继续开发；原工作区没有 `.git`，未覆盖任何车载代码。
CLI Git 代理仍重置连接，连接器仍为 Ace-Monaka-Miya，不能用其替代 kswlt 提交。

本次完成的是轨迹跟踪库、ROS 接口/节点和 Omni 恢复缓存修复；没有接通物理控制输出。

## 具体实现

### TrajectoryTracker

`src/rm_nav_control/include/rm_nav_control/trajectory_tracker.hpp`：

- admission：检查时间契约、地图/环境版本、严格递增的非零轨迹 ID、采样容量，再检查完整 polygon 与栅格。
- 按执行时间插值 pose/velocity，yaw 使用最短角；终点之后速度前馈归零。
- world-frame 速度前馈与位置误差修正相加，再旋转到 body frame；yaw 速率前馈与姿态误差修正相加。
- 横移不被丢弃，例如车头朝 +Y 时，世界 +X 对应车体 -Y。
- 对最终 XY 向量模长和 yaw 速率限幅；过大跟踪误差、非法输入、时间跳变、更新停顿立即清除轨迹。
- 完成时输出零并清除轨迹，恢复需要新的 ID。被拒绝的替代轨迹不会回退到旧轨迹。
- 环境更新可以重新验证当前轨迹；为保守起见也检查已走过的部分，后方新障碍可能导致停止。

这是外层位姿跟踪；下游 Omni PID 继续负责实测速度修正，YawManager 负责原协议的角目标转换。
无实测速度时保持已有纯前馈模式，不把状态估计速度写成硬件测量。

### ROS 数据与节点

新增 `TrajectoryPoint.msg`、`TimedTrajectory.msg`、`TrajectoryEnvironment.msg`。
轨迹消息 points 有 200000 点反序列化容量限制。环境快照把实际栅格和版本放在同一消息中。

`src/rm_nav_control/src/trajectory_tracker_node.cpp`：

- 默认 `enable_tracking=false`，必须显式传入 footprint polygon；没有虚构车体尺寸。
- 默认公共坐标系 `odom`、底盘 `base_footprint`；trajectory、grid、odom 必须一致，odom child 必须是指定底盘。
- 只接受平面导航四元数，不偷偷把完整 chassis 6DoF 当成 base_footprint。
- 使用源时间和 steady-clock 接收时间共同检查新鲜度；ROS 时钟暂停/倒退时清除轨迹。
- 健康、许可、odom、环境缺失或过期时持续发布零候选速度。
- map/environment 版本改变即取消；相同版本的新栅格仍重新做 collision 检查。
- 无效输入/许可恢复建立新的 admission 时间边界，延迟到达的旧请求不能借恢复重放。
- 输出 `/nav/trajectory_cmd_vel`，不是串口入口；状态话题 `/nav/trajectory_status` 中 enum 约定
  `0 Idle, 1 Tracking, 2 Complete, 3 Invalid, 4 VersionChanged, 5 ClockFault, 6 Diverged`。

### Omni 恢复缓存故障

首次联合测试实际复现了旧 `test_no_feedback_timeouts_and_rejection` 失败：
ready=false 期间 fresh command 仍能进入 safe_，tick 虽然会周期清除，但最后一次接收可能发生在 tick 之后；
下一次 ready=true 后，旧缓存可能短暂输出 `(0.1,0.1,0)`。

修复 `omni_pid_adapter_node.cpp`：

- 接收命令时就检查 ready、odom 及必要反馈，而非只在 tick 检查。
- ready=false、非法 odom/反馈立即清除 safe_。
- ready 恢复或超时恢复建立 source-time epoch；恢复之前生成的命令不得再次采用。
- 保持 my_serial_py wire protocol、话题字段和限幅参数不变。

原失败日志保留 `log/development_20261013/tracker_failure/omni_timeout_first_failure.log`。
新增专门回归：未就绪期间发命令，恢复时不再发新命令，确认始终零输出；延迟旧包也不能恢复。
这解决上位机缓存问题，不证明旧串口协议能使下位机旋转停止。

## 构建、测试与证据

本机 WSL Ubuntu-24.04 / ROS 2 Jazzy，真实执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /root/adam_nav_local
colcon build --base-paths /mnt/c/Users/Admin/Desktop/新导航框架/rm_nav_v2/src \
  --packages-up-to rm_nav_control --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --base-paths /mnt/c/Users/Admin/Desktop/新导航框架/rm_nav_v2/src \
  --packages-select rm_nav_control --event-handlers console_cohesion+
colcon test-result --verbose
```

三个相关包 interfaces/planning/control 构建通过。
控制包五项 CTest：command_synthesizer、trajectory_tracker core、trajectory_tracker ROS、omni_pid、omni ROS。
轨迹 ROS 回归 3 个场景；Omni ROS 回归 4 个场景。断流、版本改变、障碍、错误 frame、旧时间、许可恢复均覆盖。
轨迹回归 Domain 198，Omni 回归 Domain 199，localhost discovery；全部话题重映射为 `/test_only/...`。
只启动对应测试节点，没有串口或下位机替代进程。

联合离线验收入口：

```bash
RM_NAV_LOCAL_BUILD_ROOT=/root/adam_nav_local bash tools/local_build_test.bash offline
```

本轮最终记录与输出保留 `/root/adam_nav_local/reports`，副本 `log/development_20261013/tracker_reports`。
最终联合结果：9 个离线包构建通过；25 tests、0 errors、0 failures、1 skipped；MapBundle 回归 8 项及 smoke 通过。
跳过项仍为可选 libscrc 原库对照，协议 golden bytes/CRC 测试正常执行。

## 接线与未验证边界

1. TDT OSQP 后端仍需上游依赖，不能用本轮跟踪器冒充轨迹生成器完成。
2. 生产快照适配器需将实际 Costmap、版本和时间绑定为 TrajectoryEnvironment；本轮没有创建虚拟生产环境发布器。
3. `map` 路径与 `odom` 状态不能直接混用；需经真实 map→odom 在指定源时刻转换，或提供同一 frame 下的真实状态。
4. 候选速度还需单一来源选择、当前障碍/terrain/制动检查，再进入 motion_gate、Omni、my_serial_py。
   不自动重映射到 checked/safe，保留 MPPI 基线，防止两个控制器争抢输出。
5. HardwareInterface 的实际读写、真实反馈与下位机停车语义仍未完成。本机缺相关依赖，未声明通过。
6. 碰撞重验证目前在 ROS 回调线程执行且有预算，需真实大地图测时；超时会停，但不能宣称满足最终实时性。
7. 无实车跟踪误差、延迟、制动或资源指标；所有测试几何和健康消息明确为 TEST_ONLY。

## 提交状态

尚无本轮 commit/push，main 不变；保存两份代码补丁及一份说明补丁，基于核对过的远端文件与前轮已应用补丁。
恢复 kswlt Git CLI 后先 fetch/status，在 feature branch 审查并分步应用，中文提交，push 后 ls-remote 核实。
