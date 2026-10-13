# 建图后端本机验收（2026-10-13）

检查基线：`fc3567a`，分支 `audit-p0-tf-costmap`。只修改后端边界与验收，不替换 KISS/GICP/GTSAM。

## 发现与修复

- 录制器与 PoseGraph 已统一为每段最多 500 帧，超限明确拒绝；旧 5000 帧说法不再成立。
  保留原始 archive，下一段使用新 session；目前不提供跨段全局拼接，因此不能宣称 5000 帧全局优化完成。
- 离线优化原先保存所有完整点云。现在相邻配准只保留上一帧，回环只加载当前候选对，重建仍逐帧读取原始数据。
  大图仍限制 500 poses、最终地图 100 万体素，不静默截断 archive。
- overlap 已使用 small_gicp KD-Tree，但原实现做了 1 cm 体素预处理，会改变阈值边缘的采样集合。
  改为直接建立 KD-Tree，完整保留旧 stride 与 0.2 m 距离指标。固定种子测试结果完全一致：
  overlap=0.0185，单次本机 Release 测量 brute=5.87046 ms、KD-Tree=1.0448 ms；不作为车载性能保证。
- Hessian condition 与点云 PCA geometry ratio 分开配置，默认分别 1e-5、1e-4。
- GTSAM 降低目标函数后继续检查每条相邻/回环边残差、各帧位姿修正和最大局部形变。
  对最大残差回环做一次有界移除对照；敏感性过大拒绝发布。保留 Huber 因子。
  该对照不是全体 leave-one-out，也不能证明重复结构一定正确；真实场地回环仍需验收。
- 回环最多 2000 候选、单候选 5 s、阶段 300 s；超预算中止，禁止输出部分正式地图。

## 测试

`colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release`、`colcon test`、
`colcon test-result --verbose` 在 WSL Ubuntu 24.04 / ROS 2 Jazzy 执行。
PoseGraph 测试覆盖 100/300/500 帧、501 帧拒绝、错误回环拒绝及移除对照；固定种子 overlap 与暴力算法精确比较。

`python3 tools/smoke_offline_mapping.py`：真实 VGICP/GTSAM 原始 archive 回归通过。
`python3 tools/smoke_loop_enabled_mapping.py`：合成往返场景通过真实 KISS/GICP/GTSAM，接受 2 条回环。
工具现在把 archive 放在 Linux 原生 `/tmp`，保留测试目录。此前放在 WSL `/mnt/c` 时，
DrvFS 不支持所需 `renameat2(RENAME_NOREPLACE)`，产生了明确失败；生产原子提交没有降级绕过。
首次失败的 archive 与 `.partial` 产物保留在仓库 ignored `log/offline_mapping_*` 中。

资源指标由 optimizer 标准输出记录 `keyframes/peak_rss_kib/cpu_s/wall_s`。
100/300/500 测试是图容量测试，尚非同规模真实点云性能测试；不可用图测试冒充完整现场性能。

`python3 tools/benchmark_mapping_capacity.py` 另行运行了实际 archive→VGICP→GTSAM→地图写入，
每帧为明确合成的 1500 点，关闭回环；501 帧明确拒绝且没有输出目录。

| 帧数 | 峰值 RSS KiB | CPU s | 墙钟 s | 输入字节 | 输出字节 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 100 | 29128 | 0.467835 | 0.502120 | 1867124 | 126384 |
| 300 | 28980 | 1.344230 | 1.387060 | 5601658 | 343024 |
| 500 | 30072 | 2.172160 | 2.234940 | 9336219 | 558217 |

本次证据目录 `/tmp/mapping_capacity_zq_s_d2w`；点密度远低于实机，不可外推现场内存与耗时。

新风险：更严格的后验验收可能拒绝需要大修正的真实数据；应调查数据/外参/回环，不自动放宽门限。
提交与远端结果见本轮交接记录及 Git 历史；本分支不自动合并 main。
