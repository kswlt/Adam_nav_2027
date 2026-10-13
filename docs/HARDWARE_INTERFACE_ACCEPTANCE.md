# ros2_control ROS transport 软件验收

`RmSerialHardwareInterface` 现在有实际订阅、状态读取和命令发布路径；不再是始终 ERROR 的占位骨架。
它不打开串口，my_serial_py 仍是唯一 wire codec / IO owner，原 RX26/TX54 格式不变。

接口按 URDF 顺序定义为 vx/velocity、vy/velocity、yaw/position，状态和命令均检查接口名及唯一 joint 名。
状态初始化为 NaN，只有实际 `DriveFeedback` 才能赋值；绝不把 command、EKF 或静止假设填成测量。

默认 `allow_hardware_activation=false`。显式放行后仍须新鲜真实反馈、drive_ready 和 stopped=true 才能 activate。
activate 的 yaw target 从当前测得 yaw 初始化，不能默认写 0 使底盘旋转。
独立 command lease 必须在本次激活以后生成并严格递增；lease/feedback 超时、非法值和序列重放都会停车请求并锁存 fault。
恢复真实输入不会自动恢复旧命令，需要 deactivate/activate。XY 最终向量限幅，yaw 角目标最短角并限速。

| 入口/出口 | 默认话题 | 契约 |
| --- | --- | --- |
| measured feedback | `/hardware/drive_feedback` | DriveFeedback，真实源 stamp、sequence、measured_twist、yaw、ready、stopped |
| command lease | `/hardware/command_lease` | Header，frame=base_frame，持续新鲜且单调 |
| candidate velocity | `/hardware/ros2_control/cmd_vel` | Twist，XY，未直接接原串口入口 |
| candidate yaw | `/hardware/ros2_control/cmd_yaw_angle` | Float32，degree，原协议语义 |
| stop request | `/hardware/stop_request` | Bool，请求不是实际停车反馈 |

接口采用 Jazzy 兼容 on_init(HardwareInfo)/export 接口，在当前 ros2_control 4.48.1 可编译；
该兼容 API 有上游 deprecation warning，未伪装成新版本 API。
回调在专有 executor 上由 read/pump 消费，DDS queue depth=1；没有阻塞串口线程或持锁等待网络。
最终实时性仍需实机资源验收，普通 ROS 发布不等于 hard realtime。

## 测试

真实执行 WSL Ubuntu 24.04 / Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
source /root/adam_nav_complete/install/setup.bash
colcon build --packages-up-to rm_nav_hardware --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select rm_nav_hardware
colcon test-result --verbose
```

`hardware_transport_test` 通过 pluginlib 加载实际插件，调用实际 lifecycle/read/write/export API。
Domain 197、LOCALHOST，所有话题 `/test_only/hardware/*`，不启动串口。
覆盖：缺反馈拒绝激活、NaN 初值、真实消息到 state interface、候选输出、lease 超时、锁存恢复、
XY 限幅、非法负时间及真实反馈断流。

## 实车边界

旧 my_serial_py RX 没有轮速/twist/停车字段，因此本轮没有发布生产 DriveFeedback，也没有重新解释 reserved 字段。
真实硬件适配方必须提供已有传感器/固件的可证明测量来源；没有这些输入时 HardwareInterface 按设计拒绝激活。
用户暂缓反馈时保留已有纯前馈 Omni 软件路径，不谎称 ros2_control 闭环已经通过。

原协议没有 yaw disable，发布零 XY 和当前 yaw 只能表达保持请求，不能证明旋转已停车；
stop_request 需下位机独立 watchdog/失能机制落实并实测确认。没有真实停车确认被伪造。
