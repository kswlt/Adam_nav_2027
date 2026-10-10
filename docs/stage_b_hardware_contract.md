# 阶段 B：STM32 硬件状态接口审查与兼容契约

审查对象：`src/my_serial_py/my_serial_py/protocol.py`、`serial_bridge.py`、
`launch/serial.launch.py` 和原版 `serialpy_node.py`。当前协议保持原版兼容，未修改报文。

## 已确认的现有协议

### ROS 到 STM32

当前发送频率为 50 Hz，完整 TX 包 54 bytes：

| ROS 话题 | 类型 | 当前含义 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 只读取 `linear.x/y`，作为底盘平移命令 |
| `/cmd_yaw_angle` | `std_msgs/Float32` | yaw 目标角，单位 degree，保持上一值 |
| `/cmd_stance` | `std_msgs/UInt32` | `running_state` |
| `/region` | `std_msgs/UInt8` | 区域编码 |
| `/big_yaw_aligned` | `std_msgs/UInt8` | 大云台对齐标志 |

协议把 `x/y` 取反后写入两个 float，并把 yaw 角重复写入两个 float 字段。原协议没有
`angular.z` 角速度字段，不能把 Nav2 的角速度直接塞进 yaw 角目标。当前 8 个 reserved
float 固定为 `1.0`，不能擅自当作速度反馈。

`cmd_vel_timeout` 默认 0.3 s。超时、NaN、串口写失败和断线重连时，平移命令归零；
断线重连不会恢复断线前缓存的平移命令。yaw 目标不会因平移 watchdog 自动变成角速度或被推断为停车确认。

### STM32 到 ROS

当前接收包 26 bytes，包含裁判状态、HP、RFID、`contact_angle` 和 `is_fire`：

```text
game_type, game_progress, remain_hp, max_hp,
stage_remain_time, bullet_remaining_num_17mm,
outpost_hp, base_hp, rfid_status, contact_angle, is_fire
```

`contact_angle` 是原协议中的大小 yaw 差角，不能当作绝对云台角；`is_fire` 也不是底盘健康或停车确认。

## 当前缺失的真实状态字段

以下字段在当前 26-byte RX 包和现有 ROS bridge 中都没有真实来源，因此阶段 B 不能宣布硬件闭环完成：

| 缺口 | 需要的定义 | 当前状态 |
| --- | --- | --- |
| 轮速 | 每个轮子的方向、单位、时间戳、编码器计数或速度 | 未提供 |
| 实际底盘速度 | `base_link` 坐标的 `vx/vy/vz/wx/wy/wz`、源时间、有效标志 | 未提供；不能用 EKF 估计替代 |
| 绝对云台角 | 关节名、弧度单位、零位、正方向、时间戳和有效标志 | 未提供；`contact_angle` 不满足 |
| 停车确认 | 下位机确认已停止、确认时间、误差或状态码 | 未提供 |
| watchdog 状态 | 超时阈值、触发状态、清除条件、STM32 重启计数 | 未提供 |
| 急停状态 | 独立急停输入、复位条件、硬件优先级 | 未提供 |
| 底盘模式回执 | 当前运行/禁能/故障/手动接管状态 | 当前 `/cmd_chassis_mode` 被忽略 |

在这些字段确认前，`/hardware/measured_twist`、`/hardware/gimbal_joint_states` 和停车确认都必须保持无发布者，不能发布恒定零值或导航估计值冒充真实反馈。

## 兼容方案

1. 保持现有 26-byte RX / 54-byte TX 报文不变，继续用于裁判状态和现有底盘命令。
2. 如果固件可以扩展，增加独立版本化状态帧，包含 magic、version、sequence、source timestamp、字段有效位、轮速/底盘速度/云台角/停车/watchdog/急停和 CRC。新帧与旧帧并行接收，旧固件仍可只发送原 RX 帧。
3. `my_serial_py` 增加独立状态解析器和 ROS 适配话题；不修改原有 `/cmd_vel`、`/cmd_yaw_angle` 语义。
4. 速度单位固定为 SI（m/s、rad/s），云台角固定为弧度；协议适配层显式处理原始单位和方向。
5. 只有收到新版本状态帧且 sequence、时间戳、CRC、字段有效位和 watchdog 状态均通过时，才允许发布 `measured_twist` 或 `gimbal_joint_states`。
6. 速度、停车和急停回执分别进入恢复事务和 Motion Gate；串口连接成功本身不等于底盘停止，也不等于可以运动。

这部分需要 STM32 固件字段和现场协议确认后才能实现。当前没有修改协议，也没有把任何默认值提升为生产状态。

## 已完成的软件证据

```bash
ROS_DOMAIN_ID=88 python3 tools/smoke_serial_pty.py
python3 tools/check_my_serial_protocol.py
```

PTY 覆盖：26-byte RX、54-byte TX、噪声/分帧/CRC 重同步、字段映射、XY 符号、yaw 角保持、
reserved 字段、NaN 拒绝和平移超时停车。它使用伪终端，不连接真实 STM32，不能作为 HIL 或停车验收。

## 阶段 B 判定

软件协议兼容：**通过**。

真实硬件状态闭环：**BLOCKED**。

解除 BLOCKED 需要用户提供或现场确认：STM32 状态帧定义、轮速/底盘速度来源、云台绝对编码器来源、
watchdog/急停行为和实车抓包。获得这些信息后，再实现薄适配层、HIL 测试和停车/断连验收。
