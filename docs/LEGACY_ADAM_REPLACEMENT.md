# 不改下位机协议，替换 Adam 上位机导航

目标是继续使用车上原来的电控协议，只替换上位机的定位、规划和速度生成。
本轮仅在本机 WSL 测试，不连接 ASUS、不打开车辆串口、不启动或修改原车导航。

## 实际复用什么

已重新阅读用户提供的 `adam深北莫开源改版代码-上场版`：

- `fake_vel_transform/src/fake_vel_transform.cpp:70-105`：LIO 衍生 odometry yaw → R(-yaw) 旋转 xy。
- `my_serial_py/my_serial_py/serialpy_node.py:1499-1706`：串口参数、收发结构、缓存 yaw、定时发送。
- `pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py:145-149,761`：独立发布 `/cmd_yaw_angle`、`/region`、`/big_yaw_aligned`；它也会发布 `/cmd_vel`。

仓库保留的 `src/my_serial_py/my_serial_py/serialpy_node.py` 与用户原版文本相同；它是源代码参考，实际运行入口仍是精简后的 `serial_bridge.py`。
原通信说明中接收格式和发送包长有旧笔误，本次以实际 struct 和源代码为准。

| 项目 | 原版实际代码 | 新兼容入口 |
| --- | --- | --- |
| 波特率 / 数据位 | 115200 / 8N1 | 相同 |
| 发送周期 | 20ms，50Hz | 相同 |
| TX | 0xAA，54字节，`<BffffBBB8f` + 小端Modbus CRC16 | 相同，复用现有 codec |
| RX | 0xA5，26字节，`<BBHHHHHHIfB` + 小端RM CRC16 | 相同，保留裁判/角度/开火字段 |
| xy 变换 | R(-LIO yaw)，随后打包取负 | 上游已旋转，适配器不再旋转，打包只取负一次 |
| 两个 yaw float | 同一 `/cmd_yaw_angle`，单位度 | 仍原样复制两次 |
| running_state | `/cmd_stance`，默认3；忽略 `/cmd_chassis_mode` | 相同 |
| region / aligned | 独立uint8，默认0 | 相同 |
| 8个预留float | 默认全部1.0 | 相同，没有借位加新协议 |
| `/cmd_vel.angular.z` | 未写入上述协议 | 相同；本模式不向 MCU 添加角速度字段 |
| contact_angle | 接收并发布，不参与原 xy 旋转 | 新参考模式同样不依赖它 |

所以无需为了这个上位机替换新增电控字段、修改结构体或更换 CRC。
这证明的是协议与正常输入下的语义兼容；当前车载固件源码、自旋作用点和实际运动尚未现场核验，不能仅凭报文相同承诺所有动态行为完全一致。

## yaw 怎么处理

这里容易混淆的是“当前雷达朝向”和“发给电控的 yaw 目标”：

1. LIO yaw 用来把导航 xy 转到水平雷达/yaw坐标系。
2. `/cmd_yaw_angle` 是原业务节点给电控的角度字段，仍由原业务控制源提供。
3. 不把 LIO yaw 写进 `/cmd_yaw_angle`，不把它冒充机械编码器。

此前新串口桥默认 `yaw_policy=require_fresh`：无新目标或目标过期就停止 TX；仍保留该默认行为。
新增的 **`yaw_policy=legacy_cached`** 才复现原版：启动使用原默认0度，收到真实业务目标后缓存，持续发送最后一个目标。
**0度是原来的字段值，不是“禁用yaw”或“让下位机随便转”的通用指令。** 电控是否在普通区域自主扫描，是原电控对 region/yaw/aligned 的既有逻辑，不能从上位机代码单独证明。

兼容适配器不发布 yaw、姿态或区域目标，保留这些原话题。如果继续使用原 region/yaw 业务节点，需要单独接入它们，而不是同时运行两套串口。
本轮没有搬运旧 region_monitor 的区域判定与地形业务，也不把它已接入作为测试结论。

## 为什么使用独立速度话题

```text
Nav2/TDT → motion_gate → /nav/cmd_vel_safe                 [nav_reference]
           sensor_reference → /nav/sensor_cmd_vel_shadow  [sensor_yaw_reference]
           legacy_adam_adapter → /hardware/legacy_adam/cmd_vel [原Adam Twist语义]
           my_serial_py → 54字节原协议 → 电控
```

my_serial_py 在这个入口将 `/cmd_vel` 订阅重映射到 `/hardware/legacy_adam/cmd_vel`。
这样旧 region_monitor 或别的节点继续发布全局 `/cmd_vel`，也不会越过新模式的定位/碰撞/任务健康门。
原 `/cmd_yaw_angle`、`/cmd_stance`、`/region`、`/big_yaw_aligned` 和所有 RX 话题保持不变。
新的传输健康话题为 `/hardware/legacy_adam/serial_healthy`，只表示收到CRC正确的新报文，不表示轮速或刹车已反馈。

## 本机使用与测试

默认只看候选命令，不打开任何串口：

```bash
ros2 launch rm_nav_bringup legacy_adam_transport.launch.py
ros2 topic echo /hardware/legacy_adam/adapter_status --once
```

此时需要已有雷达参考状态链和任务许可；无许可时候选 xy 为0。disabled 模式仅省略串口健康要求，不发布虚假健康数据。

整链路虚拟串口验收会自己创建 `/dev/pts/...`，模拟电控回传并读取实际 TX：

```bash
cd /root/adam_nav_sensor_reference
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ROS_DOMAIN_ID=206 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 tools/smoke_legacy_adam_transport.py
```

测试包括：默认报文字段、独立 yaw 缓存、四种雷达朝向、正负号只处理一次、旧全局速度隔离、RX断流、LIO断流、质量失效、许可撤销、命令超时，以及恢复不重放旧速度。
原严格模式另用 `tools/smoke_serial_pty.py` 回归，保证默认 yaw 失效停发行为未被兼容模式改变。

`tools/capture_legacy_serial_vectors.py` 只提取原版 TX 方法，使用原 `libscrc==1.8.1` 在内存中生成黄金报文；不初始化原节点、不打开设备。
协议单测逐字节对照这些原版实际输出，而不只是用新编码器自校验。给定相同 xy/yaw 字段时字节一致；旧 C++ float yaw 与新双精度计算可能有舍入差异，不宣称整条浮点运算链在所有角度下逐位相同。

## 将来怎么切车

ASUS 开机后，再检查实际配置、确认原程序释放串口、验证收发和受控小速度动作。本轮不执行这些步骤。
入口预留 `transport=physical`，必须同时指定实际串口路径和 `enable_physical_output=true` 才会打开物理端口；默认不开启，也没有 `/dev/ttyUSB0` 自动回退。
新旧串口不能同时占用同一设备；不会为了部署新程序去修改或终止原程序。

仍需现场确认：

- 原电控收到 xy 后是否仍按 yaw 参考坐标解释，既有 yaw/region 来源是否完整。
- 控制的平移速度作用于底盘中心还是偏心雷达/yaw轴点；左偏16cm时，不能忽略自旋引起的 `ω×r` 差异。
- xy归零只证明平移请求归零；整车自旋停止、轮子实际停止和下位机 watchdog 行为仍需真实验证。

新增的上位机超时、数据有限性检查和串口归属检查有意保留，避免复刻原版无限缓存旧速度等缺陷；这些不改变报文格式。

## 本轮结果

基线：`b3e4be91de5622ed20cd809e79b45fe95490a337`，开始前 clean status 并 fetch 远端，未覆盖既有修改。

- 本机全14包 `colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过。
- `colcon test --executor sequential --return-code-on-test-failure` 及 `colcon test-result --verbose`：56 tests、0 errors、0 failures、1 skipped（默认环境可选libscrc）。
- 单独测试虚拟环境安装原 `libscrc==1.8.1` 后，协议单测14 passed，无跳过。运行时不新增这个依赖。
- 新整链路 PTY 测试通过，实际TX约50.8Hz，54/26字节、坐标/单次负号、yaw缓存和业务字段、旧速度隔离及所有故障归零检查通过。
- 原 `smoke_serial_pty.py` 严格yaw模式回归通过：未收到实际yaw目标不发送，目标超时停发、CRC/噪声/分包恢复、原字段及命令超时均通过。
- 首次 `ros2 run` 暴露工作树通过rsync传入WSL时脚本CRLF导致 `python3\r` 启动失败；已按 `.gitattributes` 统一脚本LF后重跑通过。Git中的脚本本来按LF存储；后续同步也必须遵守这一约定。失败日志保存在本机独立工作区 `legacy_transport_failure_evidence`，未删失败数据。

机器可读结果见 [legacy_adam_20261021.json](evidence/legacy_adam_20261021.json)。修改集中在串口yaw策略开关、新私有速度适配/launch与测试；`protocol.py` 字节编码完全未修改，原来源文件保持原样，完整底盘默认profile也未改变。

新增风险是显式 legacy_cached 会像原版一样持续刷新缓存yaw，因此需要保留原业务目标来源，不能把它解释成yaw停用；这个风险不影响默认 require_fresh。xy超时归零仍保留，完整停车/物理串口与偏心补偿没有验收。切换应在车辆停稳时完成，本轮未执行车辆端操作。

提交与远端状态见本次 Git 提交及 HANDOFF；不合并 main。
