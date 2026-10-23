# Foxglove地图与ASUS操作台

**最新UI补齐：** 已增加“取消当前任务”，状态区新增任务状态和影子速度；驱动启动前检查已有Livox进程并拒绝竞争。取消请求返回不等于动作结束或实车停车确认。详见[任务控制补齐记录](UI_TASK_CONTROL_20261023.md)。

**先看最新版：** [逐步操作、当前限制和代码审查](CODE_DELIVERY_REVIEW_20261023.md#ui真实使用步骤)。
本地场地地图已发布；下文旧PID、隧道在线、尚未审核等是历史记录，不代表当前状态。
车上原导航运行时，不要点击本操作台的建图/导航启动按钮：它会启动另一套雷达驱动。

**最新流程：** 操作台已增加“2a · 俯视检查与可通行区域”和“2b · 影子导航目标”。
地图选择→生成检查图→根据真实场地圈选→填写检查人并确认发布→保存并停止当前建图→
启动影子导航→健康通过后输入目标。详情、实际地图与测试见 [本地地图检查与定位复验](Z_MAP_REVIEW_20261023.md)。
已有本地场地自由区域审核与发布；新Z链仍未完成物理输出和自主到点验收。

## 当前布局的含义

中间 `/backend_node/runtime_map` 是Z后端以优化后位姿重建的历史累计图，约2秒更新。
不是原始瞬时扫描；当前扫描另看 `/localization/lio_world_cloud`。
右侧 `/backend_node/localization_state` 是Z后端状态；`/localization/reason` 是框架桥接的定位状态说明；
`/tf` 是动态坐标变换；`/sensor_reference/odom` 是偏心补偿后的导航轴参考里程计，不是实际底盘朝向。

- NORMAL：当前后端正常；pose_trusted=true：后端接受当前位姿。
- tf_frozen=false / tf_published=true：坐标变换没有冻结、正在发布。
- epoch/generation：重置与会话代次；pose_revision：位姿结果修订编号。
- reason中map=odom：本次建图以启动坐标建立地图，不是官方场地坐标对齐，也不是冻结地图定位验收。
- 左上MS是3D渲染耗时，不是传感器到屏幕的延迟。

本次实际检查15719点累计图：intensity最小/最大均0；z约-0.183～6.616m。
默认强度着色因此呈一片紫色。不能由此推断原始雷达强度异常或地图质量。
新布局明确设置z高度、Turbo配色、范围-0.3～7m、点尺寸2。
超出此范围会饱和到端点颜色，后续根据场地调整；不修改点云坐标或LIO参数。
只有颜色变化不能判断地图准确性，仍需看墙体重影、轨迹和独立测量。

## 已部署的操作台

ASUS workspace仍是 `/home/asus/nav_2027_vehicle_config_164/z_migration_20261023`，Domain98。
HTTP只监听ASUS `127.0.0.1:8878`；Windows通过SSH隧道访问同端口，使用随机操作令牌。
无新增Python依赖；终端使用真实POSIX PTY及本地打包的xterm.js6.0.0（保留MIT许可证）。

本机桌面“新导航框架/Foxglove操作台”内：

- `Adam_Z_operator.json`：累计地图与操作台组合布局，导入方式和刚才相同。
- `Adam_Z_operator-0.1.0.foxe`：不含令牌的可分发扩展包。
- `Adam_Z_console.url`：本机私有直达链接，当前SSH隧道已启动；不要公开此文件。
- `一键连接ASUS操作台.cmd`：重连时双击，输入ASUS密码；启动操作台、建立隧道并打开页面，不保存密码。

扩展已经写入本机 `~/.foxglove-studio/extensions/kswlt.adam-z-operator-0.1.0`。
重新加载Foxglove后导入组合布局，可在Foxglove内部使用同一个操作台。
本机私有安装副本已配置当前连接令牌；共享包和共享布局不含令牌。
若使用共享包安装到另一台电脑，在面板地址栏填写本机SSH隧道地址与操作令牌。

桌面操作被用户Escape中止，本轮没有继续点击或重启Foxglove。
**原生Foxglove扩展实际加载、iframe展示及高度配色的屏幕效果尚未验证**。
若账户/版本要求扩展开发者席位，请按Foxglove正常授权使用；不绕过许可。
独立网页操作台使用同一后端，HTTP/PTY/ROS链已实际验证。

## 按钮怎么用

1. **开始新建图**：输入新的地图名称。启动driver、mapping、foxglove；已有状态链或建图时会拒绝重复启动，不自动丢弃当前地图。
2. **保存当前地图**：停车后点击。调用原`z_nav save`，先停止/排空前端，再检查真实保存服务返回。保存成功后前端不再继续采集。
3. **停止建图**：先保存再停止；只向本工作区记录并核对启动时间的独立进程组发送SIGINT。不强制停止其他工作区。
4. **导入地图检查包**：选择保存后的Z原生地图，调用现有import_z_map生成draft。不自动宣称通行区域已审核或提升为正式地图。
5. **启动影子导航**：输入已发布且绑定Z原生地图的MapBundle路径。必须先退出建图；沿用launch中的完整地图检查。**不会打开物理串口**。
6. **实时日志**：切换driver/mapping/navigation/state/foxglove，显示真实日志末尾。
7. **打开终端**：在ASUS上创建真实bash，预加载Domain98。支持键盘输入、持续输出、Ctrl+C和显式关闭。它具有asus用户的普通shell权限；关闭网页不自动停止建图。

进程“运行中”只证明进程存活，ROS状态另外显示接收时间；超过3秒会标记过期。
当前mapping包含状态链，所以独立state进程显示未运行是正常的。
保存/导入可能耗时，操作期间按钮禁用，末尾显示实际返回或错误。

## 从源码部署和重连

ASUS（已有本次代码及雷达JSON）：

```bash
export NAV_Z_WORKSPACE=/home/asus/nav_2027_vehicle_config_164/z_migration_20261023
bash "$NAV_Z_WORKSPACE/src/Adam_nav_2027/tools/z_operator/start_server.bash"
```

Windows另一终端，若不用一键脚本：

```powershell
ssh -N -L 127.0.0.1:8878:127.0.0.1:8878 -p 2233 asus@192.168.1.164
```

操作令牌在ASUS `runtime_live_20261023/operator_token`，仅用于本机操作台。
扩展构建：进入 `tools/z_operator/foxglove` 后 `npm ci`、`npm run build`；
然后运行 `python tools/z_operator/install_local_panel.py` 安装本机副本。
直接在另一台电脑连接时雷达驱动配置必须与真实设备一致，不复制旧IP假设。

## 本轮测试与边界

代码检查基线 `03e52a4e5f06cc868089c4c0e07131f8508330c9`。

- 最初8项操作台测试通过；后续加入地图检查/目标/时钟修复后，ASUS操作台**17项通过**，既有地图发布**9项通过**，见上方最新报告。
- UI脚本 `node --check` 通过；扩展 `npm run build` 通过。
- 真实API/SSH隧道/PTY smoke通过，终端读到Domain98和后端NORMAL。
- driver3966、mapping4269、foxglove4270保持原PID，未重启原建图；未发送运动命令。
- 修复测试时发现的退出问题：rclpy默认接管SIGTERM会让HTTP残留；观察线程禁用ROS信号接管，主进程负责退出，增加退出回归。
- 未在当前建图上实际点击保存/停止/模式切换，以保留用户当前采集；相应按钮不能记作实车流程验收通过。
- 本轮只有独立操作工具与布局改动，未改定位算法、协议或ROS包代码，未重跑全量colcon。
- Windows一键脚本需要交互密码，本轮未代填密码测试；原生Foxglove加载状态如上，不能写成屏幕验收通过。

真实证据：[operator_live_smoke.json](evidence/z_localization_20261023/operator_live_smoke.json)。
新风险：shell交互具有普通用户权限、网络断开影响UI可用性；map目录内存量文件不代表已发布可导航地图。
正式MapBundle仍需按原流程检查自由空间和地图一致性。
