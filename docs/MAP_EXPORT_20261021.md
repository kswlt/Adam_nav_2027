# 当前场地地图导出：检查完成，正式导航发布受阻

用户选择本次建图坐标用于当前场地调试，不要求比赛官方坐标。此次没有创建假的官方对齐，也没有启动导航或发送新运动指令。

## 实际检查结果

原始会话：`/home/asus/nav_2027_vehicle_config_164/runtime_20261021_sensor_reference/keyframes/session_1792568087261746637_4085448541`。

- 原档案16帧全部保留；仅停止本工程capture组件，防止继续混入异常帧。雷达、状态链、Foxglove和串口组件未改动。
- 第0到1帧时间相隔4774.399秒，位置从约(0.003,0.003)变成(-4626.836,-1126.821)米，位移4762.076米。后续约0.1秒产生数百米位移。这是估计值异常，不是真实运动真值。
- 在线后端已锁存`LIO adjacent jump exceeds online mapping limits`，保留的快照只有第0帧、2806点、0回环；不能把Foxglove显示的保留地图当成本次全部运动的建图结果。
- 停止capture后实际执行离线VGICP/GTSAM入口，返回1：`Adjacent seed jump too large`。没有放宽门限或拿纯里程计替代失败约束。
- 该拒绝运行墙钟0.16秒、峰值RSS29460 KiB，仅代表早期拒绝路径，不是完整优化性能。
- 根因仍未确认。不能仅凭此判断是遮挡、硬件、时间戳、IMU、前端状态还是安装关系；本轮没有声称修复LIO。

证据：[关键帧位置检查](evidence/map_archive_inspection_20261021.json)、[实际导出清单及在线状态](evidence/map_review_export_20261021.json)。

## 已生成的文件

ASUS目录：

```text
/home/asus/nav_2027_vehicle_config_164/runtime_20261021_sensor_reference/map_exports/retained_review_20261021
  map_bundle.json
  online_status.json
  optimized_cloud.cdr
  optimized_path.cdr
  maps/retained_map.pcd
  maps/occupancy.pgm
  maps/map.yaml
  maps/terrain_reference.json
```

本机也已下载到桌面新导航框架的`output/map_export_20261021/retained_review_20261021`。

这是**检查包**，不是正式导航MapBundle。坐标仍为mapping_odom，未重命名为map。manifest明确：

```json
{
  "status": "draft_mapping_review",
  "coordinate_frame": "mapping_odom",
  "official_alignment_applied": false,
  "navigation_ready": false,
  "publishable_for_frozen_localization": false
}
```

导出使用完整`/mapping/optimized_map`，不用降采样preview冒充完整快照；要求地图和轨迹源时间相同、帧数和点数与状态相符。保留原始ROS CDR快照，输出采用新目录原子提交，不覆盖原档案。该快照只有1帧，不代表完整场地或真实回环验收。

占据栅格5cm分辨率，障碍筛选范围显式指定为mapping_odom中z=-0.1到1.5米，仅供检查，未证明实际地面高度。467个占据格、183993个未知格、0个自由格；没有把点云没看到的地方涂成可通行。terrain为每格最低观测z，不是已验证地面或可通行地形。

## 命令与测试

新增`tools/export_online_map_review.py`：只订阅ROS地图、轨迹、状态，既不打开串口也不发TF/速度。输出始终是不可导航的检查包；不是绕过现有发布器的替代入口。

```bash
# 加载当前ASUS环境后，在本次release根目录执行；输出必须是新目录。
python3 tools/export_online_map_review.py \
  --output /实际新目录/review \
  --min-z -0.1 --max-z 1.5 --resolution 0.05

python3 tools/test_export_online_map_review.py
```

本机及ASUS各3项新增测试通过：PGM上下翻转/占据未知语义、最低观测z、非有限/无效/超容量输入拒绝；真实ROS导出成功。本机原有MapBundle契约3项通过，发布器9项中8通过、1项Linux symlink检查跳过。没有修改ROS包C++或launch，本轮未重跑全包colcon构建测试；不把之前56项结果冒充新执行。

## 正式当前场地导航仍需完成

1. 查明LIO异常，验证真实点云/IMU/逐点时间及连续运动；另起新会话，保留此次失败档案。
2. 连续采集当前房间/场地，完成实际回访，检查重影、轨迹和回环。不能用当前单帧检查包替代。
3. 使用已验证的当前场地坐标定义，完成地面高度检查与有来源的自由空间生成/人工审阅。
4. 当前生产发布器/启动契约只接受官方对齐及官方自由空间mask；本次选择的是本地场地，尚需增加明确的local_site发布/启动契约，独立记录来源，不能把official_alignment_applied改成true蒙混通过。
5. 静态地图定位回放/现场定位验证后，才发布可导航Bundle并连接Nav2。本次正式发布状态为BLOCKED（输入LIO异常）；没有通过修改manifest宣称完成。

提交SHA与远端状态见本次Git提交；原始MCAP、归档和失败日志保留。此新增导出工具与报告不改变车载执行控制。
