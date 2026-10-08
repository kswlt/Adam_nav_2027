# my_serial_py 协议与虚拟串口验收

2026-10-09，在 asus / ROS Jazzy 执行。没有打开物理串口。

原版 `serialpy_node.py` 保留，SHA256 为
`c2109d37ed4b8192f611630a10dcffc43cfeae34849a64bf2d8fcbc2e1d9162e`。
运行入口转为 `serial_bridge.py`，协议独立到 `protocol.py`。
接收 RM CRC16 和发送 Modbus CRC16 分开实现，摆脱运行时 libscrc 依赖。
保留原版负 xy、重复两次 yaw 目标（度）、姿态/区域/对齐 uint8、八个 1.0 float 槽位。
继续忽略 `/cmd_chassis_mode`，继续使用红方前哨/基地字段映射。

新增行为：平移速度命令 0.3 s 超时、非有限速度、串口断开均清空 xy；重连不恢复旧速度。
接收与发送对串口对象加锁，跨包噪声、错误 CRC、断帧可重新同步。
接收大块数据每次最多 4096 bytes，帧同步后缓存少于 26 bytes。
线程退出可中断重连等待，不再逐帧 INFO 输出。

## 复现

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/my_serial_py/test/test_protocol.py -q
ROS_DOMAIN_ID=88 python3 tools/smoke_serial_pty.py
```

单测覆盖 CRC 标准校验值、原版接收查表对比、54 bytes TX 固定报文、26 bytes RX
字段顺序、连续包、逐字节断帧、噪声/错误 CRC 恢复以及非有限发送数据拒绝。
附加测试可使用原版 libscrc 1.8.1，比较 100 组固定随机输入的完整发送报文。
libscrc 仅用于兼容性核验，不是新节点的运行依赖。

PTY 测试启动真实 ROS 串口节点，用伪终端模拟 STM32，覆盖：

- 启动时零平移速度；CRC 错误和未完成帧不发布状态。
- 断帧补齐后发布六类原版话题，核对 HP、弹量、比赛状态、RFID、角度、开火字段。
- 指令以原版符号、yaw 和预留槽位打包，`angular.z=99` 不改变 yaw 目标字段。
- 停止发布指令后 xy 归零，yaw 目标保持；NaN 速度导致零平移报文。

当前协议不含绝对大云台编码器角、轮速或 IMU，也没有 yaw 角速度字段。
因此本 watchdog 是平移停车机制，不能宣称全底盘安全停车或替代下位机 watchdog。
真实下位机抓包、断线重连和串口写失败的硬件验收仍待完成。
