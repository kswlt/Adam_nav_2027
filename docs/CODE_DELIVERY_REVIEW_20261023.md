# 2026-10-23 仓库级交付审查与UI操作说明

## 检查基线和范围

基线55e717f9ed49d72047946d99eb1117e3f8c08d92，开发分支audit-p0-tf-costmap。
开始前git status、fetch已执行；远端main为69842dd76184c41a12ec0f446cdbcb9dffa1c31f。
现有未提交tools/smoke_local_site_navigation.py保留，未修改/提交。
本轮检查覆盖14个本仓库ROS包、关键launch/控制/串口/UI源码、依赖集成和验收记录；
不是逐行证明全部算法正确。138个Python源文件AST解析、14个package.xml解析均通过。
上游Z依赖在独立工作区，不能把本仓库14包与此前包含依赖的18包混淆。
ASUS读取已有colcon test-result：364 tests/0 errors/0 failures/1 skipped；本轮未重跑构建、
单测、MCAP或实车测试。语法通过和已有测试不能代替动态导航验收。

## 结论

可交付开发联调版本，不能声明新Z链已经完成实车自主导航验收。
主要代码已存在；当前缺口集中在稳定性、生产启动组合、实际车辆接口验收、任务UI与交付文档。
没有TODO不等于模块可交付。当前操作台的启动入口明确是影子导航，点击目标不会让车走。

## 模块检查矩阵

| 模块 | 实现检查位置 | 结论/缺口 |
|---|---|---|
| rm_nav_interfaces/pb_rm_interfaces | msg、srv、action依赖 | 实际消息接口存在；下位机接口变更仍须同步协议证据 |
| my_serial_py | serialpy_node.py、test_protocol.py、test_contact_angle_stamped.py | 原协议、超时和角度接口存在；新Z链端到端车辆执行、停车、重连仍未验收；setup.py许可仍TODO，分发前应澄清来源许可 |
| rm_nav_frames | config/vehicle_164_yaw_mid360.draft.yaml | 完整chassis标定仍draft；C→G未知，15.34cm偏心是批准的调试拟合，不能冒充实测 |
| rm_nav_sensors | mid360_cloud_guard_node.cpp、observation batch | 字段/几何/时间检查存在；偶发源年龄334ms/云间隔506ms仍未解决 |
| Z集成（bringup） | z_localization_frontend/mapping、bridge、sensor_reference | 新默认LIO/回环/先验定位入口存在；动态、旋转、长期和恢复精度未独立验收 |
| rm_nav_localization | resolver、EKF投影、旧frozen/recovery | 完整底盘链/旧恢复实现保留；不能与Z后端同时发布map→odom；旧测试不等于Z恢复通过 |
| rm_nav_registration | small_gicp/KISS后端与验证器 | 实后端及测试存在；当前Z默认不沿用全部旧配准入口 |
| rm_nav_mapping | 旧recorder/offline/online、Bundle | 旧录制与离线优化均显式限制500帧，原5000/500不一致已不是当前事实；不适用于Z原生长图，两个格式不能混用 |
| rm_nav_perception | scan/环境链及mppi_baseline launch | 实际Costmap输入存在；当前场地起点静态障碍/膨胀占用，真实规划返回205；地面与车体回波还需对应实测 |
| rm_nav_planning | TDT backend、轨迹/扫掠足迹验证 | 实现及独立测试存在；比赛速度、窄道、动态障碍与实车时延未验收 |
| rm_nav_control | tdt_omni_controller、omni_pid_adapter、yaw_manager | 独立Omni PID/YawManager存在；当前sensor_reference profile只用TDT平移跟踪并关闭omni adapter，不是“最终PID已接到新Z车上” |
| rm_nav_hardware | rm_serial_hardware_interface.cpp | 实HardwareInterface和反馈/激活检查存在；用户暂不提供轮速/停车反馈时该后端不能按合同激活，不应填虚假反馈 |
| rm_nav_bringup/Supervisor | task_supervisor.py、sensor_reference_navigation.launch.py | 目标/规划/取消失效联锁存在；新Z断流/重定位后的真实任务恢复仍需整链验收 |
| rm_nav_sim | 理想底盘fixture与smoke | 可验证软件控制链；不代表真实运动精度、制动或下位机watchdog通过 |

## 缺口按优先级列出

### P0：仍阻止实车成品交付

1. **偶发输入延迟未解决。** lio_input_guard.py/InputContinuity明确按250ms源年龄、500ms间隔锁停。
   多次实际出现330/334ms IMU年龄和506/757ms云间隔。短时重启恢复不是根因修复。
   复现和指标见Z_MAP_REVIEW_20261023.md及evidence/z_localization_20261023/。
   capture_live_input_latency.py可继续采集；180秒无触发只说明该窗口未复现。
2. **缺少新Z链的完整实车到点证据。** 当前sensor_reference_navigation.launch.py显式
   enable_omni_adapter=false、enable_physical_output=false。legacy_adam_transport.launch.py已有
   disabled/pty/physical三模式，不是没有串口适配代码；仍缺正式组合启动、串口唯一所有者、
   方向/速度/停止及Z故障联锁的整车验收。绝不能把旧1m/s点动结果折算为新链验收。
3. **几何与包络仍未定版。** vehicle_164_geometry.yaml中yaw_axis_offset_from_vehicle_center_xy=null，
   reference_geometry_verified=false。sensor_reference_profile.py保留半径0.54m圆包络，原依据含旧安装理解。
   当前不能证明它恰好覆盖真实整车；需要确认轴心相对车体中心并校核包络，不能直接缩小绕过占用。
4. **当前目标测试环境不满足通行。** 已确认半米内有物体；静态图23个近车障碍格、全局膨胀99，
   START_OCCUPIED有依据，不能当作规划器代码错误。需到开阔位置再测，且必须落在人工确认自由区域内。

### P1：产品操作与部署缺口

5. **不能自动防止跨工作区驱动竞争。** server.py start/alive只管理自身PID；start_mapping/navigation
   会调用start(driver)。ROS Domain隔离不能隔离同一雷达UDP配置或串口设备。
   启动前应增加跨工作区设备占用检查/明确切换；本轮不修改或停止车上原导航。
6. **缺少任务取消与全过程展示。** UI有goal输入，没有单独取消目标按钮；观察器没有订阅
   /nav/task_status或路径/速度数值来呈现任务结果。停止navigation是结束进程，不等于任务取消产品流程。
   SubmitGoal accepted只表示开始异步规划，不等于路径成功、运动开始或到点。
7. **地图选择及审核体验有缺口。** server.maps列出所有map_bundle.json，包括draft；启动端会拒绝draft，
   但下拉列表未分状态。圈选前端错误可能被轮询覆盖；提交文件保存不等于页面自动恢复未提交多边形。
   当前发布工具只生成local_site，不是官方比赛坐标发布器。
8. **桌面Foxglove内嵌面板未完成屏幕验收。** 扩展包/组合布局存在且有构建记录，不能保证当前桌面版已加载。
   独立网页是已验证入口；两者都依赖ASUS HTTP服务与本机SSH隧道。
9. **不是一键生产交付。** 缺当前车辆经验证的开机恢复、版本部署/回滚和故障运行手册整体验收；
   一键连接只恢复操作台和隧道，不等于ROS自启动，也不会自动打开物理输出。
10. **文档存在历史状态混杂。** README/操作台指南先前仍写“尚未审核”，实际已发布local_site；
    早期PID、地址145、旧LIO与历史矩阵不能作为当前启动依据。本轮在README和UI指南增加最新说明。

### P2：场地和增强功能

- 比赛官方坐标/CAD对应点尚无交付依据；当前local_site恒等对齐合理，但不是官方对齐。
- terrain_reference是minimum_observed_z，traversability_verified=false；不能视为地形可通行能力。
- 新Z系统独立真值ATE/RPE、长期负载、真实旋转平移/云台扫描、断流恢复、动态障碍和制动距离待验收。
- ROG-Map/ESDF、动态预测等增强能力不能因目录存在或技术方案列出就宣称完成；当前不作为基本导航已通过项。

## 当前现场运行说明

2026-10-23约22:35只读检查：ASUS SSH正常，重启约1.5小时；8878/8765无监听，
本项目ZLIO/backend/Nav2未运行。正在运行的是容器/root/ros_ws下ROS Humble原导航：
small_point_lio、pb2025_nav_bringup、my_serial_py。本轮没有停止它或启动竞争驱动。
已发布地图目录仍在：maps/live_20261023_navigation_1792731241655189666/map_bundle.json。
当前UI不在线不能推断代码丢失；需恢复操作台和SSH隧道。

## UI真实使用步骤

1. Windows桌面“新导航框架/Foxglove操作台”双击“一键连接ASUS操作台.cmd”，输入密码。
   打开http://127.0.0.1:8878/，保留连接进程。此步骤启动HTTP，不启动新导航或串口。
   当前原导航运行时只连接查看，不点击建图/导航启动，避免另起驱动。
2. 状态区driver/mapping/state/navigation/foxglove是本工作区进程存活，不是功能通过。
   mapping包含状态链，所以建图时独立state未运行正常；导航健康false在建图模式是预期。
3. 新建图：先完成设备切换，底盘/云台停稳，填唯一英文/数字地图名，点“开始新建图”。
   初始化完成后遥控采集；Foxglove固定map，看runtime_map、optimized_path、loop_edges。
4. 保存：停车后点“保存当前地图”，等明确成功；它会排空/停止前端，不能继续本会话采集。
   再点“结束/切换运行模式→停止建图”。只点停止不会自动保存，关闭页面也不会自动停止ROS。
5. 检查发布：选刚保存的原生目录→生成检查图（prepare会按需导入draft；不必重复点已存在draft的导入）。
   核对分辨率和障碍高度；默认-0.10..1.50m是地图z值，不保证自动等于离地高度。
   黑=障碍，灰=未知，青线=历史轨迹；绿只表示你画的审核区域，不能覆盖黑障碍。
   依次点至少3个顶点→完成当前区域→填实际检查人→勾选确认→发布。非法/自交区域会拒绝。
6. 用已发布图导航：不需要重复建图、导入和发布。停止mapping，填已发布MapBundle路径，点“启动影子导航”。
   它启动driver、foxglove、state和navigation；不接通物理串口。
   等定位NORMAL且/localization/healthy、/state/sensor_reference_healthy、/lio/input_healthy都true且新鲜。
7. 目标输入：x/y为map绝对坐标，单位m，不是前进距离；yaw单位rad。当前translation_only不负责
   用目标yaw转动车体，云台/底盘旋转仍按既定电控分工。点“发送影子目标”只启动异步规划。
   用/nav/task_status和日志判断进展，不能把accepted当成功；界面尚无完整任务进度/取消按钮。
8. 导航可视化：prior_map=固定先验图，lio_world_cloud=定位后的当前扫描；global/local_costmap看障碍及膨胀；
   /plan看全局规划（实际话题以当前ROS图为准），/nav/tdt/trajectory和/nav/sensor_cmd_vel_shadow看候选控制。
   runtime_map约2秒更新是历史优化图刷新，不是当前扫描的端到端延迟。
9. 终端：点“打开终端”创建ASUS真实bash（预加载Domain98），输入命令，Ctrl+C终止当前前台命令。
   与本机PowerShell不同；具有asus普通shell权限。实时日志下拉选择state/navigation/mapping/driver。
10. 常见错误：连接拒绝→HTTP/隧道未启动；draft拒绝→选已发布manifest；START_OCCUPIED→检查整车包络/障碍；
    health false→看三个reason，不能只重启制造通过；无点云→查驱动、frame/TF和所选话题。

## 推荐收尾顺序

先补跨实例设备检查和任务取消/反馈UI；继续定位输入延迟；校核真实安装/包络；
再在开阔已审核区域完成Z定位→规划→影子速度→物理串口→到点停车→故障恢复整链验收。
用户暂缓速度/停车反馈时可以保留明确的前馈调试路线，但不能宣称ros2_control真实反馈闭环完成。
本轮只审查和更新文档，没有修复上述全部缺口，也没有启动/修改原车程序。
