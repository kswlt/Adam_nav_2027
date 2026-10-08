# rm_nav_hardware

硬件层的第一阶段实现是原版 `my_serial_py`。本目录记录新框架对硬件的边界，不在第一阶段复制或替换串口协议。

```text
rm_nav_control
    -> /cmd_vel
    -> /cmd_stance
    -> /cmd_yaw_angle
    -> /region
    -> /big_yaw_aligned
        -> my_serial_py
            -> STM32
```

后续如果接入 `ros2_control`，`RmHardwareInterface` 只能作为薄适配层，必须保证串口收发只有一个 owner，并继续兼容现有下位机报文。

当前未实现：轮速状态、底盘 IMU、云台编码器到 `ros2_control` 的统一硬件接口。这些输入需要先确认下位机是否已经提供，不能用导航估计值伪造。

本包只定义硬件边界；实际当前通信节点仍是同一工作区内的 `my_serial_py`。任何新的控制器只能发布现有 ROS 话题，再由 `my_serial_py` 发送给 STM32。
