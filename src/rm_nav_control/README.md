# rm_nav_control

Controller baseline, Omni PID and yaw authority.

真实 ROS 入口 `motion_gate`：checked 命令经定位健康、运动许可与时间/数值检查后
输出 safe 命令。默认停车，失效清空缓存，恢复必须接收新命令。
见 [输出门验收](../../docs/motion_gate_acceptance.md)。Omni PID/YawManager 尚未实现。

`CommandSynthesizer` clamps world-frame velocity commands and emits zero during a safety gate. It does not own serial transport or strategy policy.
