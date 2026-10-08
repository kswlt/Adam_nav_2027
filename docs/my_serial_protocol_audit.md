# my_serial_py Protocol Audit

审计对象：原版 `my_serial_py/serialpy_node.py` 当前有效实现。

## 当前确认内容

- ROS 节点：`serial_node`
- 串口：`/dev/ttyUSB0`
- 波特率：`115200`
- 发送周期：`0.02 s`（50 Hz）
- 接收帧头：`0xA5`
- 发送帧头：`0xAA`
- 接收 CRC：RoboMaster CRC16 查表算法
- 发送 CRC：`libscrc.modbus`
- 接收格式：源码当前为 `'<BBHHHHHHIfB'`
- 接收完整长度：`1 + 23 + 2 = 26` 字节
- 发送格式：`'<BffffBBB8f'`
- 发送完整长度：`52 + 2 = 54` 字节（按 Python `struct.calcsize` 和当前有效源码）

## 必须保持的 ROS 接口

输入：

- `/cmd_vel`
- `/cmd_stance`
- `/cmd_yaw_angle`
- `/region`
- `/big_yaw_aligned`

输出：

- `/referee/robot_status`
- `/referee/game_status`
- `/referee/all_robot_hp`
- `/referee/rfid_status`
- `/contact_angle`
- `/is_fire`

## 已发现的文档差异

旧版 `串口通信说明.md` 将接收格式写成 `'<BBHHHHHHIIB'`，而当前有效源码使用 `'<BBHHHHHHIfB'`。两者长度相同，但字段语义不同：源码包含 `float contact_angle`，不是第二个 `uint32`。

旧版说明还将发送包记录为 39 字节 payload、41 字节完整包；当前有效源码的 `'<BffffBBB8f'` 实际为 52 字节 payload、54 字节完整包。该长度必须和下位机接收结构体及抓包结果进一步确认。

本阶段以当前运行源码和下位机实际协议为准，不擅自修改报文。下一步需要使用下位机结构体、抓包或现场回放确认字段定义，然后同步修正文档和回归测试。

## 迁移原则

1. 不改变帧头、字段顺序、长度和 CRC。
2. 不把 yaw 字段合并或去重；当前发送格式中的两个 yaw 槽位必须保持兼容。
3. 不让新的控制器直接访问串口；所有控制命令继续经 ROS topic 进入 `my_serial_py`。
4. 通信包只负责传输和基础映射，不承担定位、规划或策略逻辑。
