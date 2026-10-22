# 建图时看累计点云

## Foxglove怎么选

**看当前建图覆盖范围：**导入仓库内 `src/rm_nav_bringup/config/foxglove/sensor_reference_mapping.json`。3D的显示参考系选 `odom`，勾选 `/visualization/live_map_preview`，先关闭 `/livox/lidar`、`/lio/deskewed_odom_cloud`、`/localization/odom_submap` 的显示。

这三个容易混淆的话题分别是：原始当前扫描、去畸变当前扫描、滚动局部子图。它们只显示近期范围是正常的，不应把它们当累计全图。

累计话题每条消息已经带有本次会话的全局预览。在该话题点云设置中关闭跨消息的Decay累积（Decay time设0，显示最新一份地图），不需要把持续时间调到几百秒。Foxglove按话题最新消息显示，Decay用于额外保留多帧；重复堆积全图会额外占用浏览器内存。[Foxglove官方3D说明](https://docs.foxglove.dev/docs/visualization/panels/3d)、[官方关于无Decay时显示最新点云的说明](https://foxglove.dev/blog/compressed-point-cloud-support-in-foxglove)。

**看回环修正后的地图：**启动本项目capture和online后，导入 `mapping_online.json`，显示参考系选 `mapping_odom`，勾选 `/mapping/optimized_map_preview`、`/mapping/optimized_path`、`/mapping/loop_graph`。该预览覆盖后端当前接纳的全部关键帧，回环后更新历史点位。完整体素图 `/mapping/optimized_map`更大，通常不用同时打开。

`live_map_preview` 是快速odom累计，不会回溯修改旧点的LIO漂移；优化地图是后端地图，两者用途和坐标不同。不要强行叠在同一个坐标系后判断错位。

## 这次为什么要改代码

旧 `live_map_preview.py` 用OrderedDict实现最近访问淘汰，默认5万个体素。超过容量就删掉最久没看到的体素，因而早先建过的区域可能逐渐消失。

现在使用全局自适应体素预览：

1. 保留接收到的历史区域，没有时间过期或LRU淘汰。
2. 超过点数上限时，全局体素尺寸按2倍粗化，例如15→30→60 cm，合并同体素的显示点，保留全局空间覆盖。
3. 每条消息发布当前完整预览；ROS使用depth1的transient-local快照，晚连接的查看者也能取得最新地图。
4. 断流后保留最后一张地图，状态标为stale；不把旧图重标成新时间。

降采样不是无限精度保留所有点。细小结构会随粗化减少；检查细节应使用后端完整地图或保存的关键帧/MCAP。预览策略不改动LIO、原始录包、关键帧或优化后端。

当前累计范围是预览进程本次运行期间的接收历史，重启后RAM清空。要保留跨重启结果，保存关键帧/录包并导出地图，不能把新初始化的odom直接接到上一段地图上。

## 内存和传输量

默认仍为15 cm初始体素、最多5万点、1 Hz发布。上限控制的是显示点数，分辨率会在必要时调整。

| 显示点上限 | 一份XYZ数据 | 纯预览核心峰值RSS，WSL合成测试 | 传输建议 |
|---|---:|---:|---|
| 5万 | 0.57 MiB | 约61.2 MiB | 1 Hz，约4.8 Mbit/s裸XYZ，现有1 MiB发送缓冲可容纳单图 |
| 20万 | 2.29 MiB | 约121.1 MiB | 0.5 Hz，约9.6 Mbit/s裸XYZ，至少4 MiB发送缓冲 |

RSS包括该独立Python进程、NumPy、体素索引和临时数组，不包括ROS节点额外开销、DDS、LIO、回环、Foxglove浏览器及GPU；不能把0.57 MiB消息大小当成总内存。数据含一批最多2万点的合成扫描及一次容量粗化，不是ASUS实测，也不是所有场景的峰值上界。节点允许一帧最多20万点，极端大帧的临时开销可更大。

例如仅缓存原始XYZ，每帧2万点、10 Hz就是2.4 MB/s，10分钟约1.44 GB；再加其他字段和对象开销会更大。因此不采用浏览器无限堆原始帧。

默认轻量预览：

```bash
ros2 launch rm_nav_bringup foxglove_visualization.launch.py \
  preview_voxel:=0.15 preview_max_voxels:=50000 preview_publish_hz:=1.0 \
  send_buffer_limit:=1048576
```

更大场地需要较多细节时，停止本项目原来的Foxglove组件后再换参数，不要同时启动两个Bridge：

```bash
ros2 launch rm_nav_bringup foxglove_visualization.launch.py \
  preview_voxel:=0.10 preview_max_voxels:=200000 preview_publish_hz:=0.5 \
  send_buffer_limit:=4194304
```

前提是已部署并构建当前代码。ASUS目前不可用，本轮尚未远程部署或验证浏览器实际显示。

## 怎么判断覆盖和粗化

Foxglove的Raw Messages查看 `/visualization/live_map_status`：

- `points` / `point_limit`：当前点数/上限。
- `voxel_m`：实际显示分辨率；`initial_voxel_m`是初始值。
- `retention_policy=global_adaptive_voxel`：新策略已生效。
- `coarsening_count` / `merged_voxels`：粗化次数及合并点数。
- `evicted_voxels=0`：没有按访问新旧删除区域。
- `input_stale=true`：当前没有新扫描，不表示保留地图丢失。

如果 `voxel_m`越来越大且细节不足，增加容量或使用优化完整图；如果点数始终为0，先查LIO去畸变话题和质量/定位状态，延长Decay无法补出没有收到的地图。

## 本轮测试

基线 `41305b2`，分支 `audit-p0-tf-costmap`。开始已检查git状态并fetch远端；既有 `tools/smoke_local_site_navigation.py` 未提交修改不动。

- WSL Jazzy重新构建 `rm_nav_bringup`通过；重跑该包测试，工作区汇总64项，0 errors、0 failures、1 skipped。其余未修改包沿用上一轮通过结果。
- 单测验证容量压力后最早和最新区域都在、坐标保持、负坐标和重复体素、坏输入不能清空历史。
- 真实ROS可视化节点+明确合成输入：上限10点，触发3次粗化后保留8点、最早区域仍在；源时间不变，重复帧不接纳，断流标stale，晚订阅者取得保留快照。
- 独立进程容量/内存测试：

```bash
python3 tools/benchmark_global_preview.py --output log/preview_memory.json
```

证据：[构建](evidence/global_preview_20261022/preview_build.log)、[测试](evidence/global_preview_20261022/preview_test.log)、[汇总](evidence/global_preview_20261022/preview_test_summary.log)、[ROS行为报告](evidence/global_preview_20261022/live_visualization_report.json)、[内存原始指标](evidence/global_preview_20261022/preview_memory.json)。

本轮不证明建图几何精度，也未改在线优化算法。新风险是预览粗化会降低显示细节，且一次粗化有CPU/临时内存峰值；状态显式公布当前分辨率。中文提交推送开发分支，未合并main，未部署ASUS。
