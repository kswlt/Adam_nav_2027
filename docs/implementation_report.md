# Implementation Report

## 第一阶段任务审计（2026-10-10）

已在 ASUS 权威工作区执行 `git fetch origin`，以 feature branch `audit-p0-tf-costmap` 当前提交为审计基线。源码审查结果记录在 [phase1_function_audit.md](phase1_function_audit.md)。

审计结论是：TF、时间对齐 resolver、Point-LIO 质量门、robot_localization、GICP/KISS、MapOdomManager、Nav2 MPPI 和任务监管已有软件实现与合成/smoke 证据；真实绝对云台编码器、实测 CalibrationBundle、轮速/实际速度/停车反馈、STM32 watchdog、正式场地地图和独立急停仍未提供。因此当前状态为 **软件链可继续开发，实车运动闭环 BLOCKED**。

本次审计没有修改或覆盖 ASUS 上未提交的 `offline_graph_optimizer.cpp` overlap 优化。

## P0/P1 - Initial workspace skeleton

状态：完成本地文件落地，远端首次构建通过。

完成内容：

- 建立 `rm_nav_v2` 包目录骨架。
- 复制原版 `my_serial_py` 和 `pb_rm_interfaces`。
- 保存原版串口接口说明到 `docs/my_serial_py_interface.md`。
- 添加架构契约和依赖锁定草案。

远端验证：

- 主机：`asus@192.168.1.145`
- 系统：Ubuntu 24.04 / ROS 2 Jazzy / x86_64
- 命令：`colcon build --symlink-install --packages-select pb_rm_interfaces my_serial_py --event-handlers console_direct+`
- 结果：`2 packages finished`

尚未执行：

- 实车串口连接
- ROS launch test
- MCAP 回放

下一步：将工作区同步到 ASUS 主机，执行 Jazzy 环境静态检查和首次构建。

## P2 - Communication audit

已补充 `docs/my_serial_protocol_audit.md` 和 `tools/check_my_serial_protocol.py`。
审计发现旧版说明文档的接收格式与当前有效源码存在类型描述差异，暂不改报文，先以源码和下位机实际结构体核对。

补充发现：旧版说明中的发送长度也与当前源码不一致。当前 `'<BffffBBB8f'` 计算为 52 字节 payload、54 字节完整包；协议检查脚本已按源码更新并在 ASUS 远端通过。

## P3 - TF and calibration skeleton

已添加 `rm_nav_frames` 包、TF 所有权说明和 `calibration_bundle.yaml` 草案。当前标定值保持待测量状态，不向代码写入猜测外参。
