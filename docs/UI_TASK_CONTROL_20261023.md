# 操作台任务控制补齐（2026-10-23）

基线2e2d705，工作前status/fetch确认，保留原有未提交的smoke_local_site_navigation.py。

## 新增功能

- Supervisor新增 `/nav/cancel_task`（std_srvs/Trigger）：先撤销运动许可，再异步取消自己持有的规划和导航动作；不取消其他程序的任务。迟到的接受回调仍会请求取消，不恢复许可。
- UI新增“取消当前任务”。不要求定位健康；服务失败会明确显示。返回成功仅表示已处理取消请求，不代表动作结束或真实停车。
- 状态区新增 `/nav/task_status`、`/nav/sensor_cmd_vel_shadow`，显示阶段、原因、坐标系与vx/vy/wz；影子命令不是实测速度，过期消息仍标记过期。
- 启动新驱动前检查 `/proc` 可见Livox驱动，发现已有实例则拒绝启动，不停止原实例。ASUS实际读取到原车驱动PID24730，原small_point_lio和串口进程未改变。

检测只覆盖本机可见且名称匹配的驱动，不能发现其他主机、隐藏/改名驱动；检查与启动间仍有竞态。因此不宣称完全实现设备独占，仍需操作者协调切换。

## ASUS实际验证

- `colcon build --packages-select rm_nav_bringup --executor sequential` 成功。
- `colcon test --packages-select rm_nav_bringup --executor sequential`：17个CTest通过。
- `colcon test-result --verbose`：工作区汇总365 tests、0 errors、0 failures、1 skipped；其他包沿用已有结果，不是本轮全量重测。
- 操作台unittest：20项全部通过；`node --check tools/z_operator/review.js`通过。
- 新取消测试在TEST_ONLY Domain229/LOCALHOST执行：真实ROS服务和许可发布，注入动作句柄，覆盖三个任务阶段及迟到接受。没有实际Nav2活动目标或硬件，不算实车取消/停车验收。
- 回归覆盖外部驱动阻止启动、失效状态下可取消、服务失败传播。

## 部署与使用

已更新隔离工作区并构建，仅启动操作台HTTP，不启动新雷达、定位、导航或串口。原车Humble导航继续运行。
刷新操作台可见新按钮；访问本机8878仍需SSH隧道。新导航未运行时，任务和影子速度无消息正常。
取消服务随更新后的navigation启动，单独HTTP服务不能代替Supervisor；停止导航进程与取消任务是两种操作。

未完成：浏览器点击验收、实际Nav2活动任务取消、物理停车反馈、进度图表和地图点击目标。
偶发输入延迟、车辆包络和新Z实车到点仍待解决，本轮未放宽门限或清除地图障碍。
