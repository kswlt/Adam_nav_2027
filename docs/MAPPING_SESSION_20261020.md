# 建图模式启动记录（2026-10-20）

在用户要求后于 ASUS `.164:2233` 启动独立建图实例。最初发现原车 Docker 驱动占用雷达端口；
后续复查时其雷达/LIO 进程已自行退出、端口释放，未停止或修改原车容器、代码或启动脚本。
本次只启动本工程驱动、local_state、Foxglove、关键帧 recorder，没有启动本工程串口或物理控制。

## 运行位置

- 总目录：`/home/asus/nav_2027_vehicle_config_164`
- 运行日志：`runtime_20261020/mapping_20261021_000953/`
- 当前运行路径记录：`runtime_20261020/active_mapping_path.txt`
- Foxglove：`ws://192.168.1.164:8765`，Fixed/Display frame=odom。
- 关键帧：`runtime_20261020/keyframes/session_1792512606735936939_1931153367`

日志目录名来自 ASUS 时钟，不用于确认操作者当地日期；跨机时钟差仍需处理。
本次 PID：driver=16230、state=16318、foxglove=16630、mapping=16773。
PID 仅属于本次存活实例，停止前必须核对实际命令与进程组，不能凭旧数字停止程序。

## 12 秒实际订阅检查

| 项目 | 结果 |
| --- | --- |
| 原始点云 | 112 条，末帧 19968 点，frame=front_mid360 |
| 去畸变点云 | 103 条，frame=odom |
| odom | 102 条，child=base_link |
| 累积预览 | 11 条，末次 8763 点，input_stale=false |
| chassis_healthy / submap_healthy | 结束时均 true |
| odom→front_mid360 | tf2 查询成功 |
| 原始关键帧 | 已落盘 1 个 observation.cdr |
| WebSocket 服务 | 0.0.0.0:8765 监听 |

完整探针结果在上述日志目录的 `startup_verification.json`。
本次没有改代码/运行参数，未重复构建测试；桌面 GUI 渲染未自动验证。
静止时只有少量关键帧正常，移动达到阈值后才新增；预览不是最终优化或正式发布的地图。

如果原车导航重新启动并争用同一雷达接收端口，应先明确唯一雷达 owner，不能同时运行两套硬件驱动。
