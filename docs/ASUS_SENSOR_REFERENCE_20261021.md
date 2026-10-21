# ASUS 雷达参考模式部署与实车调试入口

## 最新状态：真实串口已开启

2026-10-21按用户明确要求，已停止本工程device-free候选组件，启动physical组件；未改动原车源码。实际串口ttyUSB0，my_serial_py进程50294；约2.013秒观察到100次成功的54字节write，约49.7Hz。xy=0、yaw缓存0°、stance=3、region=0、aligned=0。证据见 [真实TX记录](evidence/asus_serial_tx_20261021.json)。这证明内核接受串口写入，不证明电控实际动作或遥控优先级。

当前没有正式全局定位及任务许可，因此没有非零导航速度。状态查询与停止见README的“真实发送”一节。下方首次部署的“TX=0/未启动”是启用前快照，已被本条更新。

已在 `asus@192.168.1.164:2233` 独立部署；运行代码基线 `3f3bb433718630bad86fe816591064fa13868896`。
旧 `releases/20261020`、原车工作区和原程序均未修改。下面描述本次启动快照，重启或手动停止后应重新查询状态。

## 当前目录和运行状态

| 内容 | 位置 / 状态 |
| --- | --- |
| 新源码及构建 | `/home/asus/nav_2027_vehicle_config_164/releases/20261021_sensor_reference` |
| 新配置、日志、关键帧、MCAP | `/home/asus/nav_2027_vehicle_config_164/runtime_20261021_sensor_reference` |
| ROS Domain | 97；测试使用198及fixture各自私有Domain |
| 雷达 | 192.168.1.152，enp86s0 / 192.168.1.50/32；路由保持原配置 |
| 前端 | 已启动 small_point_lio，未编译或启动原版 point_lio |
| TF / 参考状态 / 子图 | 已启动，实时健康检查通过 |
| Foxglove | `ws://192.168.1.164:8765`，实际WebSocket点云接收通过 |
| 关键帧 / 在线后端 | 已启动新nav_reference会话，独立mapping_odom地图 |
| 电控候选适配器 | 已启动，device-free；候选xy为0 |
| 物理串口发送 / 自动导航 | 未启动，没有车辆速度或yaw发布者 |

当前建图会话：
`runtime_20261021_sensor_reference/keyframes/session_1792568087261746637_4085448541`。
初始快照1帧、2806地图点、0回环、0积压、fault=false、upstream_healthy=true；这只证明启动和读帧正常，不是动态回环验收。

## 已执行的验收

- ASUS 全14包 `colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过。
- `colcon test --executor sequential --return-code-on-test-failure` 和 `colcon test-result --verbose`：56 tests、0 errors、0 failures、1 skipped。
- 真实LIO源消息年龄约8ms中位数、13ms的p95；这是ASUS内部采样，不代表跨电脑GUI显示延迟。
- 原始点云、去畸变点云、传感器参考里程计、子图、实时预览均收到连续真实数据。
- TF父节点符合设计，没有观察到一个child对应多个parent。small_point_lio实际参数 `publish_tf=false`；该节点仍创建TF publisher，不能根据topic存在就判定它在发TF。
- 此版rclpy MessageInfo不含publisher_gid；只读检查器明确记录此限制，未伪造逐消息authority来源。探针兼容该字典API后通过，生产状态进程不受探针失败影响。
- 电控只读5秒：1664个CRC正确的26字节包，约332.4Hz、CRC错误0、TX=0。各反馈值当时为0，不推断contact_angle是机械角，也不把串口心跳当作轮速/刹车反馈。
- Windows实际收到mapping_odom优化预览，包含2806个点。后台8765正常监听。
- 新会话管理器在本机通过2项隔离测试：不向未标记进程发信号、重复启动不创建第二实例、只停止自身进程组。Bash脚本语法检查通过。
- 已实际启动并停止一次限时MCAP录制，验证保存流程；没有发送车辆命令。MCAP保存在本次运行目录的bags子目录，未作为带真值运动数据使用。

证据：[真实传感器检查](evidence/asus_sensor_reference_live_20261021.json)、[真实串口RX](evidence/asus_serial_rx_20261021.json)、[WebSocket接收](evidence/asus_foxglove_20261021.json)。
ASUS构建/测试原始日志位于新运行目录的 `build.log`、`test.log`、`test_result.log`。

## 现在怎样调试

1. 打开Foxglove，连接上述地址。看实时效果导入 `sensor_reference_mapping.json`、参考系odom；看后端地图导入 `mapping_online.json`、参考系mapping_odom。
2. 按README环境设置，准备移动前执行 `python3 "$NAV_RUNTIME/vehicle_session.py" start bag`，开始最多3分钟的MCAP录制。
3. 先通过遥控器低速前进约1米并回到起点，观察 `/visualization/sensor_reference_path`、原始扫描和地图是否一致。此时上位机没有运动输出。
4. 停车后停止bag，检查轨迹和观测，再进入电控速度方向与停止检查；本次没有自动执行这一运动试验。

组件后台启动/停止命令详见README；会话管理器只接受本项目marker、当前用户和匹配的进程组，记录写入有锁且原子替换，不使用pkill或killall。
命令行暂时提示“Node not found”时，可用 `ros2 node list --no-daemon --spin-time 3` 等待发现；查询topic时显式给出消息类型。该发现等待与实际数据链健康分别检查。

## 待实车验证和当前边界

- 当前优化地图依赖关键帧触发，nav_reference方向固定，因此只转yaw而不平移不会触发yaw阈值录制；实时累积预览仍更新。正式长时间扫描建图的传感器朝向触发仍需后续补齐。
- 正式场地MapBundle及全局定位未建立，不发布静态map→odom冒充通过，不开启自动导航。
- xy电控坐标、偏心16cm安装下的速度作用点、整车停止、自旋停止和下位机watchdog仍需受控实车检查。
- 没有真实角度反馈时，真实底盘朝向继续标记为不可观测；未把LIO yaw冒充contact_angle。
- 观察到的位置变化不作为静止漂移或定位真值，当前没有确认并记录可用于定量验收的人工运动距离/角度。

本次运行的是原Adam协议兼容软件，物理传输入口保持关闭；原MCAP与失败证据保留，不修改旧session标定ID。
