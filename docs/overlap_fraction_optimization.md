# overlap_fraction 优化记录

`offline_graph_optimizer.cpp` 的 `overlap_fraction()` 原先对采样后的 source 和 target
执行双层距离遍历，复杂度约为 2000×5000 次距离计算。现改为复用工程已链接的
small_gicp `preprocess_points()` 和其 KdTree：采样数量、0.2 m 欧氏半径和 accepted/total
定义保持不变，只把目标点最近邻查询从线性扫描换成现成索引。

已验证：`tools/smoke_offline_mapping.py` 的真实 VGICP/GTSAM、原始点云重建、输入不可变和
非法输入拒绝回归通过。尚未声称绝对耗时或 RSS 改善；需要在相同硬件、相同关键帧和相同线程数
下分别运行优化前基线与当前版本，再补充运行时间、峰值 RSS 和 overlap 数值差异。
