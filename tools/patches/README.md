# small_point_lio补丁选择

固定上游提交：688d75cfa780049ae532e5100ca64f46ad8b1a93。

ASUS在2026-10-22 21:30实际采用的顺序：

1. small_point_lio_raw_state.patch：原始IMU状态/世界点云发布边界。
2. small_point_lio_dense_range.patch：稠密点与滤波点共用距离/有限值过滤。
3. small_point_lio_dense_time.patch：发布点按自身时间短时投影，不修改滤波器状态。

原bootstrap仍只管理raw_state补丁。不要在旧依赖目录直接追加补丁后假设bootstrap可以原地重跑；当前部署使用独立源码、构建、安装目录，并由runtime的lio_overlay.path显式选择。旧库保留，可回退。

下面三项是**未通过真实静止回归的实验**，仅保留复现和失败证据，不启用，也不能相互叠加：

| 文件 | 实验 |
| --- | --- |
| small_point_lio_temporal_voxel.patch | 每体素选择真实返回点 |
| small_point_lio_temporal_window.patch | 1ms内位置和时间同权平均 |
| small_point_lio_mean_time.patch | 保留原空间平均，仅同权平均时间 |

它们修改同一采样函数，是互斥方案。C++几何测试通过不代表定位回放通过。完整结果、实际安装路径和回退方法见 [阶段报告](../../docs/LIO_TEMPORAL_FIX_20261022.md)。
