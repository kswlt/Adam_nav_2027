# 原版 Adam 不依赖 contact_angle 的源码核对

## 结论与此前说明的修正

原版上位机的 xy 导航命令链不依赖串口回传的 contact_angle，使用的是 LIO 衍生 odometry 的 yaw。
此前把“完整动态云台 resolver 需要真实关节角”扩展为“所有导航都必须先回传云台角”，过于绝对。
需要区分完整真实车体状态方案与原版的传感器/虚拟参考系速度接口方案；不能直接把缺少 contact_angle 当作所有模式的上车阻塞条件。

本次同时阅读用户提供的原版目录，以及 ASUS 原 Docker 的现有源码；没有修改或运行原导航程序。
这次是源码审计，没有开启串口、重启节点或执行车辆命令，也没有新增功能构建/测试。

## 原版源码可证明的内容

原版工作区路径前缀：
`adam深北莫开源改版代码-上场版/rm_nav-main/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/`。

| 文件与位置 | 实际行为 |
| --- | --- |
| `fake_vel_transform/src/fake_vel_transform.cpp:70-78` | 订阅 odometry，从 orientation 提取 yaw，传给 transformVelocity |
| 同文件 `:83-91` | 发布 gimbal_yaw→gimbal_yaw_fake，旋转取 odometry yaw 的负值 |
| 同文件 `:95-105` | xy 按 R(-yaw) 旋转；角速度加 spin_speed；没有使用 contact_angle |
| `pb2025_nav_bringup/config/reality/nav2_params.yaml:153-162` | odom_topic=odometry，真实参考参数名=gimbal_yaw，虚拟参考=gimbal_yaw_fake；cmd_vel_nav2_result→cmd_vel |
| 同参数文件 `:276`、`:471`、`:522` | Navigator/Costmap 等使用 gimbal_yaw_fake |
| `loam_interface/src/loam_interface.cpp:65-97` | 把 LIO 位姿转换为雷达里程计，初次取得固定安装变换后缓存 |
| `sensor_scan_generation/src/sensor_scan_generation.cpp:62-81` | LIO 雷达位姿结合 TF，生成虚拟导航参考的 odometry |
| `my_serial_py/my_serial_py/serialpy_node.py:1661-1664` | 缓存 /cmd_vel 的 xy |
| 同串口文件 `:1676-1701` | 发送 x_val=-latest_x、y_val=-latest_y，yaw_val 来自独立 /cmd_yaw_angle；不靠 contact_angle 旋转 xy |
| 同串口文件 `:1594`、`:1624-1626` | contact_angle 仅从 RX 解析并发布 |
| `rm_static_tf/launch/static_tf.launch.py:105`、`:137-147` | 雷达/车体/云台有静态安装约定，而非真实编码器驱动的完整动态安装链 |

原版顶层 `串口通信说明.md:54` 将 contact_angle 注释为“大小yaw差角”。
该文档不能证明它等于云台相对底盘机械角；在单 yaw 车上字段是否有业务用途仍需电控确认。
此次仅采用该字段含义记录，报文大小以实际 struct/解析器为准，不能照抄文档中的历史包长。

## 速度变换的实际意义

令 θ 为 odometry.orientation 提取的 yaw，原版代码为：

```text
vx_out =  vx_in cosθ + vy_in sinθ
vy_out = -vx_in sinθ + vy_in cosθ
```

gimbal_yaw_fake 使用 -θ 的相对旋转，意图给 Nav2 一个不跟随云台自旋的航向参考；
输出速度再转换回 gimbal_yaw 参数指定的参考系，然后交串口打包。
例如 θ=90°、输入 (1,0)，转换后约 (0,-1)，串口符号变换后约 (0,1)。
这说明原版仍使用位姿和角度信息，但角度来源是 LIO 衍生里程计，不是串口角度回传。
gimbal_yaw 在原版链里是通过固定 TF 与雷达里程计构成的导航参考，不能仅凭 frame 名把它认作真实底盘 yaw。

## ASUS 原 Docker 的只读核对

容器为 rm_win_nav_local，源码路径与打包原版不同，但关键数学一致：

- `/root/ros_ws/src/navigation/fake_vel_transform/src/fake_vel_transform.cpp:83-95,116,130-131`：
  同样从 odometry 提取 yaw，使用 -yaw 的虚拟 TF，以及 R(-yaw) 的 xy 变换。
- `/root/ros_ws/src/pb2025_nav_bringup/config/reality/nav2_params.yaml:207-217`：
  gimbal_yaw/gimbal_yaw_fake，输入变为 cmd_vel_collision_checked，输出仍为 cmd_vel；有 fake_frame_x_offset=0.07。
- `/root/ros_ws/src/hardware/my_serial_py/my_serial_py/serialpy_node.py:286-317`：
  缓存 xy，再取负发送，角度目标仍为独立 yaw_val。

这证明车载原版也有此转换链，不是只根据 README 推断。没有重新启动原版来验证物理动作。

## 仍不能由上位机源码证明的内容

本次没有找到并核实 STM32 的实际运动控制代码，因此不能断言电控内部如何进行云台→底盘转换，
也不能假定新框架以 base_link 语义产生的同名 xy 可以不转换就替换原版输出。
原版能工作的必要条件是上下位机对最终 xy 的参考方向约定一致；该约定是下一步应复用和核对的接口。

对于随 yaw 转动且旋转轴偏离底盘中心 16 cm 的当前安装：

- 使用传感器参考导航可以不在上位机恢复全部真实车体姿态，但必须明确这是传感器/虚拟参考模式。
- 避障包络须覆盖偏心后的整车轮廓，不能把车体中心的 50×50 cm 方形直接当作雷达中心的轮廓。
- 精确车体 SE3、实际 chassis 中心状态和硬件闭环仍需真实机械角/其他真实姿态来源。
- 不得把 LIO yaw 同时标成“云台机械角”和“底盘 yaw”，或直接关闭 encoder 健康门伪装完整模式已通过。
- 原版中的静态/动态同名 fake-frame 发布及 TF 失败返回 identity 等行为，不应在复用接口时原样搬入生产链。

本次只完成审计与更正阻塞判断。新框架的传感器参考兼容 profile 尚未接入，
当前动态 CalibrationBundle 仍为 draft；已有旧录制/优化暂停状态保持，未开启物理输出。
