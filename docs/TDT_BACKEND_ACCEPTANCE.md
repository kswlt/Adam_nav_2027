# TDT Polynomial Backend 软件验收

本轮从 PDF 指定的 T-DT-Algorithm-2026/tdt-nav-kit 复用 MinimumSnapOsqp，固定上游
`814d5ce0237acc7faa8c070e5c5e61e2ddc81996`，MIT 许可保留；源码集中在 third_party。
没有用端点五次插值冒充 TDT。实际使用 OSQP QP、方形 corridor、上游梯形时间分配与碰撞迭代。

`TdtMinimumJerkBackend` 接收 Path 的 XY/首尾 yaw、真实当前 vx/vy/wz、占据栅格、polygon、版本和执行时间，
输出 TimedTrajectory。六个系数是五阶多项式，第三导数目标是 minimum jerk。
Yaw 用独立标量 TDT QP 保持实际初始 wz，终点 wz=0；中间 waypoint 的路径切线不作为底盘 yaw 目标。

必要上游补丁只暴露碰撞迭代后真实 segment_times、系数及时间归一化标志，并补标准头文件；
补丁保存在 tools/patches/tdt_timing_output.patch。没有从可视化 polyline 猜测执行速度和时间。

验证包括：

- 平移/旋转网格坐标转换，未知格拒绝，完整 polygon 再验证。
- 当前 XY/wz、终点速度和位姿残差检查。
- 用导数 Bernstein 控制点的凸包计算全时域速度/加速度/jerk 上界，覆盖采样之间。
- 超约束重新分配时间并重新求解，保留实际初始速度，不通过缩放采样速度伪造满足约束。
- waypoint、采样、迭代、总时长和 wall-time 预算；失败不发布部分输出。

本机 WSL Ubuntu 24.04 / Jazzy，OSQP v0.6.3、OsqpEigen v0.8.1 源码构建通过。
`colcon build --packages-select rm_nav_planning` 通过；6 项 CTest 覆盖非零初始速度/角速度、
±pi、纯 yaw、旋转地图、阻塞 waypoint、tight jerk 重求解及预算拒绝。
三点合成输入首次接受约 0.3 ms，tight jerk 输入执行 4 次轴组求解（一次 retime）；
这不是实车地图/最大容量性能保证。

依赖安装：

```bash
bash tools/bootstrap_tdt.sh
export CMAKE_PREFIX_PATH="$HOME/.local/share/adam_nav/deps/install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HOME/.local/share/adam_nav/deps/install/lib:${LD_LIBRARY_PATH:-}"
colcon build --packages-up-to rm_nav_control --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select rm_nav_planning
colcon test-result --verbose
```

当前核心已实现并测试；ROS Path/环境快照适配与统一命令来源接线随后实施。
默认物理输出不变，没有向车辆发送运动指令。

Git 已恢复，专用 SSH 身份为 kswlt；此前六个步骤已独立中文 commit/push，远端为 fb2aef2。
本轮 TDT 改动独立提交，完整构建/实车测试不能用上述核心单测替代。
