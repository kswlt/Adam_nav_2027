# rm_nav_bringup

Launch files, profiles and NavSupervisor integration.

This package is part of the RM Nav V2 staged implementation.

运行节点 `scripts/task_supervisor.py` 通过 `/nav/submit_goal` 接受目标，调用真实 Nav2
预规划与导航动作，并只为自己拥有的唯一活动目标发布新鲜运动许可。
恢复后需通过管理节点的稳定定位、新路径/目标验证；故障取消任务，不缓存重放。
见 [任务监管验收](../../docs/task_supervisor_acceptance.md)。
`NavSupervisor` 辅助类仍是基础状态接口，不代替此 ROS 节点。

`local_state.launch.py` 从同一份显式 verified CalibrationBundle 启动绝对编码器 TF、
关闭 TF 的上游 LIO、时间对齐 resolver 与真实 EKF。未测量模板默认拒绝启动。
当前 `/state/chassis` 与 Nav2 `/odom` 尚未自动相连，见
[动态云台状态链验收](../../docs/local_state_acceptance.md)。

`config/profiles.yaml` keeps `stable_baseline` as the default. Enhanced modules are explicit opt-in and must pass the same replay and real-robot checks before activation.
