# 本地场地地图检查、发布UI与先验定位复验

检查基线：`28444126f6ec164f1ac5ce3b1e92fd5c875f11f3`，分支`audit-p0-tf-costmap`。
本轮继续已保存地图的导航交付流程；不使用虚构自由空间，不接物理输出。

## 已执行

1. 保留正在运行的Domain98建图（PID11850），driver3966、Foxglove4270不变。
2. 导入已保存原生地图`maps/live_20261023`，生成`maps/live_20261023_draft/map_bundle.json`。
3. 真实地图包含3587关键帧、15719点；PCD保留二进制XYZI原本，draft为XYZ副本。
4. 在UI生成492×413、5cm/格的俯视检查图，原点[-17.1,-11.3,0]。
   障碍投影z范围暂用当前导航scan配置的[-0.10,1.50]m，**相对本次建图原点，不是离地高度**。
   2054个障碍格，其余未知，自由格0；没有自动把空白区域当空地。
5. 页面实际加载检查图、区域圈选和目标点输入控件；只是滚动/检查，没有代用户圈选或勾选审核。

本轮实际地图预览：[current_site_review.png](evidence/z_localization_20261023/current_site_review.png)。
当前图有稀疏观测/未知区域，不能据此宣布整个场地可通行或地图完整。

## 新UI怎么用

操作台`http://127.0.0.1:8878/`经现有SSH隧道访问。

1. 在“2 · 地图与导航”选择已经保存的原生地图。
2. 在“2a”设置分辨率和实际适用的障碍物高度范围，点“生成所选地图检查图”。
3. 结合现场和3D地图检查高度/地面/障碍物。在俯视图依次点击顶点，点“完成当前区域”。
   可圈选多个区域；黑色障碍观测始终保留，灰色未圈选区域仍是未知。
4. 填写实际检查人，确认检查内容后勾选复选框，点“确认并发布本地导航地图”。
   没有实际检查时不要勾选。程序拒绝缺少确认、空区域、自交、越界和无自由格输入。
5. 复用`publish_map_bundle.py`生成占据栅格、地图YAML、观测地形参考、原生Z包副本和正式local_site manifest。
   使用本次建图坐标、恒等变换，**不宣称官方场地对齐**。不覆盖旧图，临时目录失败回滚。
   生成的terrain是minimum_observed_z，仍标记traversability_verified=false，不是已验证地面坡度模型。
6. 新manifest自动填入导航路径。完成当前采集后，先保存并停止当前mapping，再启动影子导航。
7. 重新初始化状态链并等待定位健康；“2b”输入map系x/y和yaw弧度，点“发送影子目标”。
   目标经过既有`/nav/submit_goal`服务；服务拒绝会报错，不把accepted=false当成功。
   物理串口始终未启用。

本轮尚无真实操作者确认的自由空间；**没有发布navigation_ready地图，没有启动Nav2整链，也没有发送目标/实车速度**。
UI没有绕过原MapBundle/导航健康契约。

## 先验地图定位实际测试

从现场实时驱动另录约22秒原始PointCloud2/IMU，录制进程持续25秒（包含发现话题时间）。
bag在ASUS：`runtime_live_20261023/prior_test_raw_20261023/`，原始MCAP约110.24MiB。
本机原始备份：桌面`新导航框架/实车原始数据备份/20261023_Z先验定位验收/prior_test_raw_20261023/`。
未重写任何CDR、header或接收时间；本次使用原接收顺序，无派生调度。

```bash
source "$NAV_Z_WORKSPACE/src/Adam_nav_2027/tools/z_environment.bash"
export ROS_DOMAIN_ID=226 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 tools/smoke_z_localization_bag.py \
  --bag "$NAV_Z_WORKSPACE/runtime_live_20261023/prior_test_raw_20261023" \
  --prior-map "$NAV_Z_WORKSPACE/maps/live_20261023" \
  --output "$NAV_Z_WORKSPACE/prior_test_live_20261023" --rate 1 --max-wall-seconds 120
```

实际结果：pipeline_pass=true，3849里程计、195去畸变云、3849轴参考、3472后端位姿；先验图收到1次。
Z后端接纳prior scan-to-map观测，末尾NORMAL/epoch0，健康曾通过，LIO覆盖数据末尾，map→odom唯一父关系通过。
测试进程正常退出；Domain98现场建图不切换、不重启。无真值，不报告ATE/RPE或精度合格。
这段短记录只证明已保存图的重载与近期现场数据定位链，不等于Nav2整链或长期稳定性验收。

证据：[current_site_prior_22s.json](evidence/z_localization_20261023/current_site_prior_22s.json)。

## 故障与修复

- 页面之前用Windows Date.now减ASUS接收时间，实测两机相差约10.26秒，误报“消息已过期”。
  现由ASUS单调时钟计算接收年龄，UI读取age_s；没有改操作系统时钟或放宽传感器源年龄门限。
- 操作台重启会丢失准备好的检查图，现仅缓存预览及ID；不缓存审核通过声明、不替用户确认。
- 当前现场轴参考已锁存`LIO pose motion exceeds physical continuity limits`，后端仍可NORMAL。
  **这是真实未解决故障，不能被时钟显示修复掩盖。**保持导航输出关闭，保留日志与当前会话；
  后续导航模式初始化后仍须检查参考状态健康，复发则需采集故障前后的原始数据/位姿分析。
- 最初部署命令在错误本机目录运行，上传未发生，ASUS测试缺少test_map_review模块；已纠正目录后重新部署并测试通过。
  没有把该次失败当作有效测试结果。

当前现场状态快照：[operator_review_status.json](evidence/z_localization_20261023/operator_review_status.json)。

## 验证、影响与剩余事项

```bash
python3 -m unittest discover -s tools/z_operator -p 'test_*.py' -v
python3 -m unittest discover -s tools -p test_publish_map_bundle.py -v
node --check tools/z_operator/review.js
```

ASUS操作台17项、既有MapBundle9项，全部通过。覆盖未知不变自由、障碍保留、多边形自交/越界拒绝、
明确审核要求、地图变化拒绝、目标编码和拒绝返回、单调时间年龄，以及原PTY/进程归属回归。
页面JS语法检查通过，原生地图导入/真实检查图和独立先验回放已实际执行。
复用Pillow画多边形及现有import/publish工具、Nav2任务服务与Z定位器，不新增SLAM或控制算法。
没有ROS包代码变更，本阶段未重复全量colcon；没有将尚未完成的人工审核/发布/实车步骤计为通过。

新风险：操作者圈选错误或高度过滤错误仍会影响地图；圈选并不能证明地面承载性或动态障碍安全。
现有PCD障碍保留、未知区域保留和导航健康联锁继续生效。
下一步依赖实际场地检查，不用假数据填充reviewer、编码器或停车反馈。
提交/推送分支保持audit-p0-tf-costmap，未合并main；最终提交与远端状态见本轮回报。


## 18时导航复查与操作台健康修复

检查基线664280e43caf63d34f3e59d1329575f078d7fd53，修改前fetch确认远端开发分支一致；main未合并。
此前人工圈选已经发布local_site地图，版本z-map-22b974023e5fb251，navigation_ready=true；
不是官方坐标地图，terrain_traversability_verified=false。原始自交圈选和make_valid分段证据保留。

恢复ASUS服务后首次点云间隔0.757236s触发0.5s联锁；等待驱动稳定并重新初始化后，
实际观测定位NORMAL、两项健康true、bt_navigator active。随后IMU source age=0.330381s
超过0.25s再次锁存失败，定位健康false。没有放宽门限，没有下发物理运动。
后端旧NORMAL文本被桥接器重复发布，操作台原先仅检查该文本，不能反映真实导航许可。

修复tools/z_operator/server.py：观察三个Bool健康心跳；目标入口要求localization、sensor_reference、
LIO input全部true且单调接收年龄不超过1s。已有Supervisor联锁保留。旧NORMAL不再足以放行。
兼容影响：旧操作台缺少健康发布者时明确拒绝目标。index.html保留API错误提示，避免轮询覆盖。
实际发布请求先保存，原始用户圈选留档，不自动把非法区域扩展为自由空间。

ASUS执行：python3 -m unittest discover -s tools/z_operator -p 'test_*.py' -v，18项全部通过。
新增缺失/false/过期三个健康信号的9种回归情形。真实HTTP请求在NORMAL文本仍存在时
被健康门拒绝，未调用运动服务，见operator_health_gate_live.json。
本次仅操作台Python/HTML变更，未重跑colcon build/test；此前364 tests结果不算本次新测试。

30秒只读探针观察：5332条IMU，年龄最大3.14ms，间隔最大8.64ms；原始云266条，年龄最大103.34ms；
质量门输出274条，最大间隔400.74ms。观察窗口未发现大于20ms系统时钟跳变，网卡计数无丢包。
原始云与质量门计数不同包含DDS发现及BEST_EFFORT接收差异，不能据此断言质量门复制或丢帧。
证据input_timing_1818.json；仅为故障后30秒观察，未捕获此前330ms异常的根因，不能宣称修复传感器延迟。

当前仍阻塞：持续原始数据记录并复现延迟、定位稳定恢复、真实目标路径与影子速度验收、
新Z定位链实车闭环、故障恢复。物理输出false。操作台重启恢复，ROS实例保持原有故障状态以留证。


## 18:23后持续复查：轨迹限频、原始记录和真实规划拒绝

代码发现foxglove_trace_publisher在每次高频Odometry回调中序列化整个历史Path，最多5000点。
改为源时间10Hz采样，5Hz定时序列化，只有新接受点才发布；维持有限长度、原始坐标/时间和重复消息拒绝。
不改变LIO、TF、规划或控制数据。实机进程CPU生命周期平均读数旧约40.8%、新约28.7%，
两个读数负载/生命周期不同，不构成受控性能基准，不据此宣称延迟根因修复。

ASUS首次colcon测试失败：Windows上传导致脚本CRLF，shebang出现python3\r。归一LF后重建；
新增高频测试最初按固定条数断言，测试spin实际耗时超过理想2秒，修正为按实际墙钟限制5Hz。
最终测试结果见本轮回报；失败输出在ASUS CTest日志保留。新测试覆盖400次高频输入、10Hz采样、
5Hz输出上限和停止输入后不重复发布。独立ROS_DOMAIN_ID=210，未给生产话题注入虚拟消息。

60秒新原始MCAP：runtime_live_20261023/latency_trace_1830，约591MiB，包含原始IMU/点云、质量门、
真实里程计和健康状态。报告latency_trace_1830_report.json：LIO健康2917 true/0 false，
参考点2915 true/2 false，定位2790 true/85 false。不视为全链稳定通过。
几分钟后再次出现IMU source age=0.334156s，定位锁停；见latency_recurrence_1832.json。
说明本轮仅解决可视化发布量问题，传感器/DDS/回调时延根因仍未定位，不自动恢复旧会话。

真实ComputePathToPose：当前map坐标约[0.0193,-0.1665]，目标[-0.8750,0.5250]，
planner_id=GridBased/use_start=false。返回action status6、error_code205、path_points0；
ASUS安装的nav2_msgs/action/ComputePathToPose.action确认205=START_OCCUPIED。
只调用规划器，无速度/物理发布。静态地图车体中心格为0，周围不满足完整方形净空；
自由格1972、分辨率0.05m，不能只依据中心自由就认为足迹可通过。
随后尝试对比静态/全局代价图时定位再次过期而无当前TF，未完成分层归因。
尚不能判断为静态地图、实时障碍层、车体回波或footprint参数中的哪一项，不清除真实障碍冒充修复。

最终ASUS命令：colcon build --packages-select rm_nav_bringup --executor sequential；
colcon test --packages-select rm_nav_bringup --executor sequential；colcon test-result --verbose。
构建成功，包内16个CTest通过；工作区保存的汇总364 tests/0 errors/0 failures/1 skipped。
本轮未重跑其他包，不把工作区汇总误称全量重测。
原始591MiB MCAP与metadata已完整复制到本机桌面：
新导航框架/实车原始数据备份/20261023_Z延迟排查/latency_trace_1830。


## 18:35后：静态占用分层、现场确认、有效健康提示

检查基线ba716cef5f4f3049abe2b906d701c6fceba550a9，fetch确认开发分支一致。
静态/map在当前参考点周围约0.6m方形区域有23个100占用格和146个自由格；
/global_costmap同区域23格100、146格99。即静态障碍已足以导致起点膨胀占用；
不能断言实时代价层无附加影响。真实规划再次返回205 START_OCCUPIED。
证据start_occupied_layer_comparison.json。PCD半径0.6m内179点，z约[-0.015,0.813]m，
均位于当前障碍高度范围；见map_origin_obstacles.json。
用户随后确认半米内有物体。因此本轮不清除占用、不缩小0.54m调试旋转包络来制造规划通过。
C→G仍未知，当前包络尚未实测验收；应在场地开阔位置进行后续目标测试。

新增只读tools/capture_live_input_latency.py，记录原始header、DDS发送/接收和回调时间，
有界环形缓冲、等待真实健康后捕获故障前后数据；不发布ROS消息、不改变源时间和门限。
首次误以Jazzy MessageInfo为对象，实际为dict而退出，保留首次证据；改为dict访问后真实运行180秒。
后续运行观察到真实健康，但未捕获新的锁存故障(trigger=null)，不能宣称IMU延迟修复。
max IMU age约228ms发生在发现启动时，其header到DDS source约227.8ms，DDS收到至回调约0.47ms；
不能将这段启动积压当作此前334ms故障的根因。约2.7秒quality输出间隔包含受控state重启，
不能算稳定运行断流。详见timing_capture_1837_summary.json；完整只读记录已备份本机数据目录。

修复z_localization_bridge.py有效状态说明：健康false时发布status=UNHEALTHY及session、
输入健康、心跳、pose、原子帧、后端信任/过期依赖；不再反复转发旧NORMAL作为有效状态。
健康算法、时限和上游backend原生状态话题不变。ROS回归新增断流后reason必须UNHEALTHY且说明stale。
ASUS：colcon build --packages-select rm_nav_bringup --executor sequential成功；
colcon test --packages-select rm_nav_bringup --executor sequential通过16个CTest；
colcon test-result --verbose工作区汇总364 tests、0 errors、0 failures、1 skipped。
只重跑受影响包，其他包沿用已有测试结果。已安装新代码，活跃进程需下次state/navigation重启读取。
当前不为更新提示打断已恢复的健康会话；本轮没有发送物理控制指令。

后续仍需捕获真正延迟故障、在开阔位置复测规划/影子输出、完整实车到点与故障停车。
