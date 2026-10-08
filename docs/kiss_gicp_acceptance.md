# KISS 粗配准与 GICP 精化验收

2026-10-09，在 asus 的 Ubuntu 24.04 / ROS 2 Jazzy 实际编译并调用上游算法。
`KissGicpBackend` 接收 source、经过候选区域限制的 target 和原定位初值，
调用真实 KISS-Matcher 估计 `target_T_source`，再调用 small_gicp 精配准。
不使用 KISS 的成功标志直接接受定位；结果继续经过 LocalizationValidator。

## 已验证范围

5000 个合成三维点，已知平移 `(3, -1, 0.4)` 米、yaw 1.2 rad、pitch 0.3 rad，
输入初值为单位矩阵。一次运行结果：平移误差 `1.62e-6 m`、旋转误差
`4.31e-7 rad`、欧氏 RMSE `0.001174 m`、内点率 `1.0`、耗时约 `188 ms`。
这仅是该合成数据上的算法验收，不代表实车精度、重复场景鲁棒性或性能上限。

测试同时验证空点云、NaN、非法 SE(3) 初值拒绝，以及大修正仍返回 `CANDIDATE`。
修正幅度相对于调用者原初值计算，不能被 KISS 提供的新初值掩盖。
置信度沿用 GICP 内点率代理值，不是正确定位的概率。

## 安装和复现

需要系统开发依赖 `libeigen3-dev libtbb-dev libflann-dev liblz4-dev`、CMake、Git、Python3。
这些依赖在 asus 已安装。用户目录安装，不要求脚本提权：

```bash
source /opt/ros/jazzy/setup.bash
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_small_gicp.sh
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_kiss_matcher.sh
CMAKE_PREFIX_PATH=/home/asus/nav_deps/install:${CMAKE_PREFIX_PATH:-} colcon build
./build/rm_nav_registration/kiss_gicp_backend_test
```

KISS-Matcher 固定 `e3440b6`（上游 tag v1.0.2，内部 C++ CMake 版本是 0.3.0）。
ROBIN、PMC、xenium 均固定完整提交，见 `dependencies.lock.yaml`。
脚本只在构建用 ROBIN 副本中将 xenium 的浮动 `main` 替换为固定提交；
保留上游源码仓库，未修改配准算法。系统 APT 依赖和其他尚待接入库仍未完整锁定。

## 尚待接入

纯 C++ 后端现已接入 [受限恢复 ROS 调度](kiss_recovery_ros_acceptance.md)，
支持运动上界/场地边界裁剪和不同点云的一致性候选复核。
自动丢失触发、停车确认、旧轨迹失效和重规划事务尚待实现。不得用整个比赛场地无约束搜索
替代候选区域，不得将单次 KISS/GICP 结果直接写入 map→odom。
