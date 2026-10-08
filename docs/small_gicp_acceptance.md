# small_gicp 后端验收

2026-10-09 在 asus / ROS Jazzy 完成真实 C++ 配准测试。
使用 [上游 small_gicp](https://github.com/koide3/small_gicp)
的 helper API，不修改其配准、下采样和协方差估计算法。
锁定提交：`fa0cfc983417c044b6036f71dfa448038bb9c820`（版本 1.0.1，MIT）。
依赖源码、构建和安装位于 `/home/asus/nav_deps`，不提交依赖生成物。

## 复现

```bash
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_small_gicp.sh
source /opt/ros/jazzy/setup.bash
CMAKE_PREFIX_PATH=/home/asus/nav_deps/install:${CMAKE_PREFIX_PATH:-} colcon build
source install/setup.bash
./build/rm_nav_registration/small_gicp_backend_test
colcon test --packages-select rm_nav_registration rm_nav_localization
colcon test-result --verbose
```

`dependencies.repos` 锁定该源码；`dependencies.lock.yaml` 当前是部分锁定清单，
尚未锁定的依赖仍需要补齐版本，不能称为完整可复现环境。

## 接口与度量

`RegistrationRequest` 明确传入目标点云、源点云以及 `target_T_source` 初值。
适配器调用真实 GICP，并输出变换、Hessian、下采样后点数、迭代次数及运行时间。
`residual` 为最终变换下最近邻距离的 RMSE（m），`fitness` 为均方距离（m²）。
`inlier_ratio` 分母使用下采样后的源点数，避免和原始点数混用。
`condition_score` 为 Hessian 最小/最大特征值比；它受平移/旋转单位和场景尺度影响，
当前阈值是调试门限，仍需真实数据标定。
`confidence` 当前是 inlier_ratio 质量代理，尚未校准为概率。
Validator 拒绝 NaN/Inf、负质量量、非 SE(3) 变换及非法配置；
没有质量置信度的大修正也会拒绝，不会进入待确认候选。

## 本次结果

固定随机种子生成 2000 个三维点，施加平移 `(0.12,-0.10,0.03)` m
以及 yaw 0.04 rad / pitch 0.02 rad，再从单位初值配准。
单线程结果：平移误差 `0.000013239 m`，旋转误差 `0.000006803 rad`；
RMSE `0.001443 m`，inlier_ratio `1.0`，Hessian 比值 `0.375642`。
该次适配器调用约 `6.42 ms`，不代表实车频率或规模性能保证。
空点云、非有限点、非刚体初值、无重叠点云、下采样后塌缩点云以及非法配置均通过拒绝测试。
此测试使用异常检查；现有配准断言测试也显式关闭 NDEBUG，避免 Release 下空跑。

当前完成的是后端库；ROS 点云接入、冻结地图裁剪、缓存、时间与 TF 验收、
真实场地门限、KISS 恢复仍需继续实现，不能宣称 P5 已完成。
