# 停车、取消导航、TF 提交与清图事务验收

2026-10-09，asus 上的软件链路完成本步骤。事务默认关闭，必须显式启用。
自动失定位触发、重新规划和最终运动权限恢复尚未接入。

## 启动与接口

在冻结地图定位 launch 中增加 `enable_recovery_transaction:=true`，同时需要
`enable_recovery:=true`、实际 `map_version` 和场地范围 `field_bounds`。
Nav2 使用现有 `mppi_baseline.launch.py`，必须已经激活。

`/localization/commit_recovery` 服务（`rm_nav_interfaces/srv/CommitRecovery`）
接收当前地图版本和恢复会话编号。`accepted=true` 只表示异步事务开始，不表示已提交。
状态通过带会话编号的 `/localization/recovery_state` 发布。

## 必须满足的提交条件

- 当前会话已有两份不同点云的一致性候选；确认观测年龄不得超过 0.8 s。
- `/hardware/measured_twist` 为底盘 `base_link` 坐标中的实测六维速度，所有值有限。
  源时间及接收心跳均不超过 0.2 s，重复、过期、错坐标和非有限消息不计入停车确认。
- 平移速度范数和角速度范数均不超过 0.02，连续满足至少 0.3 s。
- matcher 的 `/localization/frozen_map_valid` 为 true，接收心跳不超过 0.5 s。
- 本步骤所需 Nav2 cancel 和双 costmap 清理服务存在。

实测速度必须由实际反馈适配器提供，不能由发送的零速度命令代替。
原 my_serial_py 报文没有速度反馈；目前没有自动生成该话题，缺少反馈会拒绝提交。
阈值是调试初值，实车需结合传感器误差、制动过程和 yaw 行为调整。

## 异步事务顺序

1. 健康保持 false，发送 cancel-all 到 navigate_to_pose、navigate_through_poses、
   follow_path、spin、backup。取消成功后还要等待当前动作进入终止状态。
2. 停车反馈和冻结地图在等待过程中必须继续有效；确认过期、拒绝或总等待超过 5 s
   进入 FAULT，不提交 TF。
3. 仍满足条件时，唯一 MapOdomManager 一次性提交并发布 map→odom 修正。
4. 调用 local/global costmap 的 ClearEntireCostmap 服务，等待两者完成。
   清图失败或超时保留已提交 TF，进入 FAULT 并持续禁止运动，不自动回滚。
5. 发布 WAIT_REPLAN，matcher 使用提交后的初值返回局部 GICP。
   新鲜局部结果可以验证修正后的定位，但不解除运动暂停。

取消列表可通过 `cancel_actions` 参数配置；接入新的执行器或行为时必须加入对应动作。
现有 Nav2 模式通过取消导航和 FollowPath 使旧任务退出；未来 TDT TimedTrajectory
还需要接入轨迹版本失效和执行器确认，不能把 Nav2 取消当作所有执行器的失效证明。

## 验收

```bash
source install/setup.bash
ROS_DOMAIN_ID=92 python3 tools/smoke_recovery_transaction.py
ROS_DOMAIN_ID=93 python3 tools/smoke_recovery_transaction_faults.py cancel_rejected
ROS_DOMAIN_ID=94 python3 tools/smoke_recovery_transaction_faults.py clear_timeout
ROS_DOMAIN_ID=95 python3 tools/smoke_recovery_transaction_faults.py map_changed
```

正常验收调用真实 KISS/GICP、Nav2 NavigateToPose/cancel-all 和两个实际 costmap 服务。
先通过真实局部 GICP 建立旧 TF，再恢复较大偏差；验证旧导航目标返回 CANCELED、
修正方向正确、清图完成、局部 GICP 重新运行且仍禁止运动。
每项非法停车输入拒绝验收都使用新鲜的候选确认，避免因候选过期而产生假阳性。

取消拒绝和清图超时使用明确的服务故障替身；不是实际 Nav2 算法或成功清图的替代。
这些测试验证错误处理：提交前保持旧 TF，提交后保留新 TF，两种情况均保持健康 false。
地图变更验收验证已确认候选被作废。
所有停车反馈来自理想模型，不能作为实车已停止、通信 watchdog 或运动控制验收。

后续已接入自动受限触发、新 Nav2 规划/目标、稳定定位确认与任务运动许可，见
[新任务验收](task_supervisor_acceptance.md)。本文件的事务验收单独关闭任务监管，
验证 WAIT_REPLAN 保持；实车反馈适配器与 TDT 轨迹版本仍待完成。
