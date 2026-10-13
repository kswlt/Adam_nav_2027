# my_serial_py 传输边界验收（2026-10-13）

RX26/TX54、CRC、字段位置、平移反号、yaw 度数、保留字段均保持原版协议。
`serialpy_node.py` 原版文件未修改。新桥接器 `serial_bridge.py` 增加以下处理：

- 没有收到有效 yaw 目标时不发 TX，避免默认 0 度造成意外转向。
- 平移命令 0.3 s 超时输出零平移；yaw 0.3 s 超时停止刷新 TX。
  原协议没有 yaw disable，停止发包依赖下位机 watchdog，不能声称等价于实测急停。
- 断线清除旧平移/yaw/RX 状态，重连不能重放缓存。
- pyserial exclusive 加上 Linux `/proc` 字符设备占用检查，防止打开原导航已占用的串口。
  `/proc` 可见性受系统权限影响；启动前仍需确认原导航进程与串口所有者。
- `/hardware/serial_healthy` 仅由真实连接和近期 CRC 合格 RX 驱动。它不是 drive_ready、轮速或停车反馈。

Omni 默认仍要求 `/hardware/drive_ready` 和实测速度。用户选择暂不接轮速时，
可显式 `require_measured_feedback:=false readiness_topic:=/hardware/serial_healthy`，
运行纯速度前馈；该模式没有速度闭环，也没有停车确认，不应称为完整硬件验收。
YawManager 使用真实 `/odom` 当前 yaw。底盘与下位机 yaw 零点/正方向需现场确认。

验收：`ROS_DOMAIN_ID=195 python3 tools/smoke_serial_pty.py` 在 WSL Jazzy 通过。
真实 PTY 和模拟 STM32 覆盖分包、噪声、错误 CRC、字段兼容、启动 yaw 禁发、
速度超时、非有限速度、yaw 超时禁发、RX 断流健康失效。
本次没有打开物理串口；原始通信 fixture 原样保留。

尚未验证：真实端口重连、系统 `/proc` 权限、STM32 watchdog、yaw 方向、实际速度/停车效果。
代码提交和远端结果见 Git 历史，禁止把此 PTY 结果描述为实车通过。
