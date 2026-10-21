# 电控 yaw 源码核对：哪些角度参与导航

2026-10-21；导航审查基线 `1a6e799`。只读解压并检查用户提供的两个ZIP，未修改、编译或烧录电控。本文结论针对这份源码，未确认车上烧录固件与ZIP一致。

源码根目录简称：

- G：`Season27_Regular_Sentry_Gimbal-Omni-main`，云台板，接收导航串口并转发CAN。
- C：`Season27_Regular_Sentry_Chassis_Omni-main`，底盘板，执行轮子和大yaw电机控制。

解压原件保留在本机 `.dev/electrical_review_20261021`，未把电控整份源码上传公共仓库。下文行号均为ZIP原件。

## 先说结论

1. 上位机不读contact_angle仍能工作，是因为**电控自己已经读取大yaw编码器并转换速度**，不是因为整车不需要相对角。
2. 上传代码把`contact_angle`直接赋0；这不能证明大yaw没有转，也不是上位机解析坏了。
3. 新框架的雷达参考模式分工成立的前提是：雷达固定在大yaw侧，朝向与电控接收速度的参考方向一致；电控负责相对底盘的转换。当前源码显示这条内部转换存在，但安装轴向/正负/零点仍需实车核对。
4. **不能把LIO yaw填到contact_angle，也不能直接填到串口yaw字段。** 前者是不同物理量；后者在这版电控中可能当转速执行。
5. 旧上位机沿用“yaw角度”的命名，电控字段则叫yaw_speed；直接复制原协议，只能保证报文兼容，不能保证这个字段的控制含义一致。此前README把它一概称角度指令不够准确，本报告更正。

## 五种不同的 yaw

| 名称 | 实际含义 | 来源/使用处 |
|---|---|---|
| LIO yaw | 雷达相对启动里程计坐标的水平朝向；弧度 | small_point_lio，导航把固定参考方向的xy转到雷达水平系 |
| chassis_yaw_angle | 大yaw电机相对编码器零点的角度；度 | C/Chassis.cpp:130；零点C/robot_param.hpp:79为117.26°；用于底盘速度旋转 |
| 云台IMU yaw_cnt | 电控IMU连续累计角度；度 | G/Gimbal.cpp:134；CAN 0x105传到C，供大yaw惯性控制使用；不保证与LIO同零点 |
| contact_angle | 上传结构体注释是大小yaw差角；当前实际恒为0 | G/Auto_control.cpp:127；不是当前机械角反馈 |
| 串口第3/4个float | 旧上位机把同一个yaw数值写两次；电控名为rotate/yaw_speed | 第3个未找到生产消费，第4个转发到大yaw规划速度；见下一节 |

这里“小yaw”是源码中的命名，不能据此推定车上实际有两个yaw轴；需要确认这份工程与当前硬件/固件的对应关系。尤其不能把云台板IMU的朝向未经安装核对直接等同于雷达朝向。

## 真实的平移链路

```text
地图/odom中的期望平移速度
  → 上位机用LIO yaw旋转到雷达水平朝向
  → my_serial_py按原协议给xy各取负一次
  → 云台板UART7校验54字节报文
  → Navigation模式复制x_speed/y_speed
  → CAN 0x101转发xy
  → 底盘板读取大yaw真实编码器
  → 相对角旋转及底盘轴向转换
  → 全向轮逆解/控制
```

证据：G/Auto_control.cpp:93–109校验长度/帧头/Modbus CRC；G/UART_data_txrx.cpp:176–210实际UART7接收；G/Chassis.cpp:281–325复制xy；G/BoardLink.cpp:84–88打包；C/BoardLink.cpp:112–116解包；C/Chassis.cpp:130、147–151、207–215转换；225–228轮速逆解。

**两次旋转不重复：**上位机处理“固定参考方向→传感器朝向”，电控处理“云台侧参考方向→旋转底盘”。如果上位机再额外按机械相对角旋转一次，才会重复补偿。

对源码轴交换可直接代数核对。令上位机发给打包函数的雷达水平系速度为(u,v)，线上的xy为(a,b)=(-u,-v)，编码器相对角为q：

```text
CAN解包： chassis.vx=b，chassis.vy=-a
Navigation update： Vx_set=-a，Vy_set=-b
Navigation calculator最终：
  Vx_set = -a*sin(q) - b*cos(q) = u*sin(q) + v*cos(q)
  Vy_set =  a*cos(q) - b*sin(q) = -u*cos(q) + v*sin(q)
```

这是底盘程序内部轴变量，不把它们冒称ROS标准x/y。四轮安装、驱动正反、雷达安装角、编码器零点是否与之相符仍是实测项。保持报文原格式不能自动证明这些物理方向。

## 串口 yaw 到底控制了什么

上位机 `src/my_serial_py/my_serial_py/protocol.py:56–62`：

```text
float1=-x；float2=-y；float3=yaw_degrees；float4=yaw_degrees
随后running_state、region_code、aligned、8个float预留、CRC
```

电控G/Auto_control.hpp:224–233定义同样布局：float3叫`rotate`、float4叫`yaw_speed`。本次搜索没有找到`Navigation_receive_1.rotate`的实际读取；底盘旋转指令来自独立的模式逻辑，不能把float3当作已接通的底盘角速度。

float4的实际链路：

```text
Navigation_receive_1.yaw_speed
 → G/BoardLink.cpp:116，CAN 0x109第一个float
 → C/BoardLink.cpp:151，gimbal.yaw.planning_spd
 → C/Gimbal.cpp:145–155（LQR配置中的导航分支）
```

这个分支在Navigation模式且没有自瞄跟踪时，默认`yaw.spd_set=80.0f`；**仅当`chassis.is_aligning==5`时**改成`yaw.planning_spd`。is_aligning来自上位机`running_state`，不是big_yaw_aligned，也不是region_code。PID备选分支319–328采用同样逻辑；C/Gimbal.hpp:34当前启用LQR。

实际速度反馈C/Gimbal.cpp:85通过rad2degree转换成度/秒，所以这里规划速度也是**度/秒**；头文件写rad/s与实现不符。

| 条件 | 这份源码中的大yaw行为 |
|---|---|
| Navigation、无自瞄跟踪、running_state=3（当前上位机默认值） | 默认80°/s；串口float4=0不会让它停转 |
| Navigation、无自瞄跟踪、running_state=5 | float4直接作为度/秒速度目标；发90表示约90°/s，不是对准90° |
| 正在扫描 | 扫描分支优先，目标80°/s |
| 自瞄跟踪成立 | 进入其他惯性角位置控制分支，不能套用上述导航分支 |
| LOCK | 电机输出控制层禁止正常力矩指令 |

所以“发送零xy、零yaw”**不等于整车和云台停止**；仅声明当前上位机平移为零。上轮抓包中的running_state=3、region=0、yawfloat=0，按这份源码仍可能触发导航默认扫描。是否正在发生取决于遥控模式、自瞄状态和实际烧录固件。

## contact_angle 为什么一直是0

G/Auto_control.cpp:127明确是`stm32send_1.contact_angle = 0;`。底盘确实有编码器相对角，但当前C/BoardLink.cpp:191–195的发送表只发送裁判数据，没把该角返回云台板；云台板也没有从真实角更新上传字段。

想恢复真实chassis TF，需要另行增加“底盘相对编码器角→CAN→云台板→串口→ROS”的反馈契约，确认轴、符号、零点、单位和采样时间。不能仅把赋0一行换成LIO yaw。当前雷达参考模式不依赖这一回传；要恢复精确底盘中心/矩形轮廓导航才需要这条观测。

## 两个会影响上车的实际问题

1. **导航断流未形成完整停车逻辑。**UART7收到包后设Navigation_online=200；G/Chassis.cpp:339递减，但Navigation分支281–320没有依据该计数清零缓存xy。全User目录查询也没有找到据此门控的导航消费。上位机持续发送零xy可以清零缓存，但拔串口或上位机突然掉电不能指望缓存自动消失。这是源码审查结果，尚未做实机断流试验。
2. **源码导航模式的底盘自旋与预期不一致。**G/Chassis.cpp按区域生成input.r，但C/Chassis.cpp:147–151在Navigation模式直接W_set=0；后续Navigation calculator只改xy，没有使用vr恢复自旋。tasks中的use_follow/final_r也未进入calculator。故不能声称这份源码已经实现“导航时底盘自旋平移”；实际车若能做到，需核对是否烧录了不同版本。

遥控模式：G/remote_control.cpp:206–237显示双下或RC离线进入LOCK，双上进入Navigation，双中FLOW_Gimbal；离开Navigation后xy取遥控来源。LOCK最终效果还受两板通信和控制层状态影响，本轮没有进行硬件接管测试。

## 推荐后续处理

- 保留当前雷达参考xy链，不把contact_angle设为LIO yaw，不在上位机再补一次机械相对角。
- 先与电控确认ZIP是否就是当前烧录版本，以及雷达是否与电控转换的云台参考方向一致；用小幅轴向测试核对xy和机械角符号。
- 上位机字段应区分导航yaw速率与旧版缓存角度；当前继续保留旧格式，**暂不发送新的非零yaw或切running_state=5**。待模式契约确定，再改适配器，不把旧角度值直接当速率复用。
- 若维持电控自主扫描，上位机只提供xy；大yaw扫描速度与自瞄切换留在电控。若要上位机控制扫描，再明确速度接口、deg/s和接管条件。
- 断流停车与Navigation模式W_set逻辑需要单独修复并实车验证；本轮未修改电控或启动运动试验。

验证范围：静态追踪结构体、UART7/CAN转发、模式分支、单位与变量引用；未声称完成固件构建、真值定位或实车运动验证。无需为纯源码解释重跑ROS全包构建。
