# 逐点时间修复、静止回归及ASUS切换

检查基线cffd3f8109c9fd222a5d490e17a1902a042b5408。HTTPS fetch连接重置后改用SSH fetch成功：main仍为69842dd、功能分支仍为cffd3f8。保留用户原有未提交的smoke_local_site_navigation.py，不修改原车工作区。

## 当前实际采用的修复

保留原small_point_lio空间平均、0.2m叶长、0.2m地图分辨率和所有滤波参数。只启用：

1. 上一轮的dense_range补丁：距离/有限值过滤同时作用于发布稠密点，避免零返回进入地图。
2. 新dense_time补丁：对稠密点按自身采样时间，用当前速度/角速度作短时投影，再转入odom；不修改滤波状态或协方差、不添加第二次车体外参、不改消息时间戳。

短时投影采用上游predict_state对应的局部匀速/恒定角速度模型，不是有独立真值证明的精确运动轨迹。更大的加速度、IMU相位误差及场景动态仍可能导致误差。

补丁为 `tools/patches/small_point_lio_dense_time.patch`，叠加于原raw_state和dense_range之后。独立源码/安装目录：

```text
/home/asus/nav_2027_vehicle_config_164/lio_review_dense_time_20261022
```

实际候选构建：`MAKEFLAGS=-j2 colcon build --packages-select small_point_lio --cmake-args -DBUILD_TESTING=OFF`，通过，约33.8秒。上游Eigen编译警告仍保留。

## 拒绝启用的采样方案

没有把“采样时间更合理”等同于“整体精度必然更好”。在独立副本验证了三个方案，均未部署：

- temporal_voxel：每个体素取最接近质心的真实点，保留其原始时间。
- temporal_window：最多合并1ms内同体素的点，位置和时间同权平均，参考rose的短时间批处理思路。
- mean_time：原空间平均和数量不变，只将任取时间改为同权平均时间。

这些方案的几何/时间单测通过，但旧静止原始数据回放出现回归，因此仅作为失败证据/实验补丁保留。**这些补丁相互替代，不能依次叠加，不在默认bootstrap或运行配置中启用。**

### 静止原始数据对照

使用旧run_20261021_202129前25秒：250帧点云、5,000帧IMU，独立复制序列化原始输入，源时间及log时间不改。每次重新初始化前端，输出约23.9秒；只检查估计运动，不冒充独立定位真值。相同检查包络：最大三维估计变化<=3cm、P99步进速度<=0.1m/s、姿态变化<=0.05rad；未放宽门限。

| 方案 | 最大三维估计变化 | 末尾距起点 | 结果 |
| --- | --- | --- | --- |
| 原版本，0.2m | 2.030cm | 1.547cm | 通过 |
| range＋dense_time，0.2m，当前采用 | 2.030cm | 1.553cm | 通过 |
| 真实点代表，0.2m | 3.940cm | 3.361cm | 拒绝启用 |
| 真实点代表，0.05m | 5.613cm | 3.968cm | 拒绝启用 |
| 1ms时间窗，0.2m | 4.012cm | 2.904cm | 拒绝启用 |
| 同权时间平均，0.2m | 5.014cm | 4.588cm | 拒绝启用 |

原录制位姿与本次从原始数据重新初始化回放的窗口/初始条件不同，不能拿旧报告的1.21cm与表中2.03cm直接作算法差值。

## 完整回放仍复现离线拒绝

新增 `tools/replay_raw_mapping_pipeline.py`，仅播放原始点云/IMU，实际启动质量门、小Point-LIO、参考状态、观测适配和关键帧录制。回放后先停止录制、释放锁，再运行真实offline_graph_optimizer --enable-loops。Domain215～218，LOCALHOST、use_sim_time、/clock；不回放旧位姿/TF/健康，不启动控制节点。

本次45秒快速旋转窗口四组均产生24个新关键帧、约432～434组真实LIO及参考位姿。各组物理命令话题发布者均为0。

| 方案 | 第9→10帧RMSE | 离线结果 |
| --- | --- | --- |
| 原版本0.2m | 0.162125m | 超过0.15m，拒绝 |
| range＋dense_time 0.2m | 0.162003m | 拒绝 |
| 再加真实点代表0.2m | 0.161574m | 拒绝 |
| 再加真实点代表0.05m | 0.164229m | 拒绝 |

没有插入纯里程计桥接，没有更改0.15m门限，没有生成正式MapBundle。新完整回放与旧160帧现场采集的关键帧时刻/初始化不同，所以第9→10帧不能与旧139→140帧当作逐字节同一输入。

后端源码确认该RMSE是欧氏最近邻距离，不是点到平面误差，也不是GICP的马氏目标。扫描覆盖/点密度和去畸变都可能影响它；目前没有足够证据把拒绝归因于其中一项。下一步应检查失败帧的覆盖、密度和局部子图配准，而非继续盲调LIO噪声或放宽门限。

原版本稠密云17,044～18,435点/帧，修复后约6,400～8,850点/帧，与去除近距/零返回相符。原160帧关键帧和在线导出仍是旧发布内容，不会因切换库而自动清理。

## 测试环境修复及实际测试

- 初次独立C++测试被LD_LIBRARY_PATH中的旧同名库覆盖，2项失败。已修正测试入口，强制候选lib目录优先；通过/proc/maps或ldd核实实际加载库。原失败日志保留。
- 点时间回归使用已知匀速平移/旋转下的固定墙点，旧姿态有>10cm几何误差，按真实点时刻投影后误差<1e-10m；同时检查不改变状态、零时间一致、负时间/NaN拒绝。
- 当前采用库的真实距离过滤＋点时间CTest 2/2通过，入口 `test_small_point_lio_dense_range.bash ... tools/test_small_point_lio_dense_time.cpp`。
- 三种采样实验的C++测试通过并未替代上述真实静止回归。1ms测试曾因要求浮点除法逐位相等失败，改为1e-7m算术容差后通过；未修改运动验收门限。
- 当前采用库叠加项目安装后，在隔离Domain225重新执行colcon test / colcon test-result --verbose：64 tests、0 errors、0 failures、1 skipped。中文控制台3项测试通过。

日志、原始输出轨迹和各候选失败报告在 [证据目录](evidence/lio_temporal_20261022)。原MCAP不改写、不删除。

## ASUS实际运行切换

2026-10-22 21:30，仅重启本项目管理的state/Foxglove。原车导航、原deps/install_lio和旧工作区未覆盖。重启前只读IMU1190样本、gyro P95=0.01047rad/s、加速度模中位0.9973g，符合低运动初始化条件；不是独立静止真值。

运行目录仍为 `runtime_20261022_fit153_console`。新增可选 `lio_overlay.path` 指向上述候选的install目录；vehicle_component.bash只对state读取它，无文件时保持原行为，无效路径则拒绝静默回退。原运行脚本备份为 `vehicle_component.before_dense_time.bash`。

state管理PID93346，真实small_point_lio PID93386，Foxglove管理PID93477。进程命令行及/proc/93386/maps均确认使用dense_time库。库只包含原raw_state＋dense_range＋dense_time，**不含三个静止回归失败的采样补丁**。

15秒真实只读探针：原始点云146帧、LIO146帧、去畸变云145帧、参考位姿145帧、子图68帧、累计预览15帧；真实输入/参考/子图健康true。LIO源年龄P95=14.0ms、预览源年龄P95=53.2ms。实际稠密云8,294点，原始20,064点；未把消息年龄解释为Foxglove客户端端到端延迟。

观测到的参考XY变化范围6.35mm、LIO三维变化范围5.91mm；用户未单独确认这15秒绝对静止，不能作为静止漂移真值。Python没有GID，TF唯一authority仍未独立验收。

driver/state/Foxglove与旧完成会话的online仍运行；bag/capture/physical/navigation停止。累计预览因LIO重初始化而清空，旧mapping_odom检查包保留，未将新odom预览与旧图混合追加。

菜单和Foxglove地址不变：`ws://192.168.1.164:8765`。当前可看新实时/累计云，尚不能依据这份旧检查包启动导航。

### 回退方法

先停止本项目state和Foxglove，保持capture/physical/navigation关闭；将runtime的lio_overlay.path改为原依赖的绝对install路径 `/home/asus/nav_2027_vehicle_config_164/deps/install_lio`，再启动state/Foxglove。重初始化会重置odom及内存预览，不追加旧录制。没有删除/覆盖原库，因此可回退。
