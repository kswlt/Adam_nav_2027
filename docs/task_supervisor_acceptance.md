# 自动受限恢复与新任务放行

本步骤接入真实 ROS 节点与 Nav2 动作接口，尚未接入物理底盘。

## 自动触发

冻结地图 launch 新增 `enable_auto_recovery:=true`，默认关闭，必须同时开启
`enable_recovery:=true` 并提供测量过的 `field_bounds`。
已有可靠 map→odom、最后可靠地图位置及新鲜 `/odom` 后，连续三次质量拒绝或
合格观测超过 1 s 未更新，才启动一次受限 KISS/GICP 搜索。
启动时没有可靠位置不会自动搜索；失败不会无限重复请求。
搜索半径仍为 `max_speed * lost_time + margin`，超过上限或中心不在场内则拒绝。
`/odom` 必须为 odom→base_link，时间单调、源时间和接收时间均不超过 0.2 s。
自动触发只创建候选搜索会话，不自动调用定位提交，也不自动解除停车。

## 任务入口与权限

`mppi_baseline.launch.py` 默认启动 `task_supervisor`，由它单独发布
`/nav/motion_enable`。生产运行不要再启动其他同话题许可发布者。
导航任务通过 `/nav/submit_goal`（`rm_nav_interfaces/srv/SubmitGoal`）提交 map 坐标目标。
服务 accepted 只表示开始异步规划；状态见 `/nav/task_status`。
监管先调用真实 ComputePathToPose（GridBased），取得有效新路径，再创建新的
NavigateToPose 目标。仅当前任务拥有的唯一活动 UUID 可以获得运动许可。
直接提交外部 NavigateToPose 目标不会获得许可；外部目标抢占会撤销当前许可。
本地 `/state/chassis_healthy`、全局健康与恢复状态心跳期限为 0.25 s。
本地失效优先 STOP，取消任务，禁止以 WAIT_REPLAN 继续规划；恢复不重放旧任务。失健康、服务消失、目标结束或规划超时均保持零许可；
失健康取消当前任务，健康恢复不会自动重放它。

此处路径是 Nav2 预规划证明，Nav2 行为树仍可重新规划；并非 TDT TimedTrajectory
版本、逐点执行证明或 ROS 网络身份认证机制。

## 恢复后的放行

停车、取消旧动作、TF 提交和双清图完成后仍进入 WAIT_REPLAN。
新任务规划与新目标接受后，监管调用 `/localization/resume_recovery`。管理节点要求：

- 地图版本及会话匹配，真实 Nav2 状态中存在请求的活动目标 UUID。
- 提交之后至少三份合格 LOCAL_GICP 结果，相隔至少 0.15 s、间隙不超过 0.5 s，
  最新合格观测不超过 0.4 s；非法结果会清除稳定计数。
- 实测连续停车、冻结地图心跳与 odom 仍有效。
- map 路径时间晚于 TF 提交、年龄不超过 2 s，2–10000 个有限、单位四元数、
  场地内的点；起点距当前机器人位置不超过 0.5 m。

通过后管理节点发布 TRACKING 与健康 true；任务监管仍独立核验自身目标及新鲜健康，
才能发布许可。旧路径、错误 UUID、错误会话及没有新任务均不能解除保持。

## 软件验收

```bash
source install/setup.bash
ROS_DOMAIN_ID=96 python3 tools/smoke_auto_recovery.py
ROS_DOMAIN_ID=97 python3 tools/smoke_task_resume.py
```

自动触发验收使用真实 GICP/KISS 算法，验证超时自动搜索、双候选确认和 TF 不直接提交。
2026-10-09 已在 asus 通过自动触发和完整任务软件验收；新增后端 40 次连续热启动通过。
任务验收启动真实 Nav2 planner/navigator/controller/costmap 与监管节点，验证外部目标
无许可、恢复后新任务到达地图目标、错误放行请求被拒绝、外部抢占和失健康取消、
健康恢复无旧任务重放。负向路径只用于请求拒绝测试，成功路径来自真实 Nav2。
日志保留在 `log/task_resume_smoke.log`、`task_resume_localization_history.txt` 和
`task_resume_estimates.txt`，包含失败时的质量、收敛和迭代信息。

底盘与实测停车反馈仍为明确的理想模型；合成云的算法收敛不证明实车定位稳定性。
原串口没有速度反馈，yaw 报文也不发送 angular.z；本步骤不自动连接物理串口。
LIO/编码器/EKF 软件链已接入；真实传感器、反馈适配器、TDT 执行失效与实车回放仍需完成。
