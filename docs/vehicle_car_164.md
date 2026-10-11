# 车载电脑 192.168.1.164 实车配置

已确认车载电脑：

| 项目 | 实测值 |
| --- | --- |
| SSH | `asus@192.168.1.164:2233` |
| ROS 工作区 | 车上已有 ROS 2 Humble 工作区，尚未部署 `Adam_nav_2027` |
| Wi-Fi | `192.168.1.164/23` |
| 雷达网卡 | `enp86s0`，`192.168.1.50/32` |
| MID-360 | `192.168.1.152` |
| 串口 | `/dev/ttyUSB0`，Silicon Labs CP2102 |
| 串口协议 | `my_serial_py`，115200 baud |
| 底盘 | 全向轮，单 yaw 目标角语义 |
| 雷达安装 | 正装，车辆中心左侧 0.08 m |

已生成：

- `config/hardware/mid360_car_164.json`：Livox host/radar 网络配置。
- `config/hardware/vehicle_car_164.yaml`：车载硬件 profile。

当前 profile 保持 `partial_hardware_profile`，原因是雷达相对车体的前后位置、高度、云台零位和真实绝对编码器还没有测量。`y=+0.08 m` 采用 REP-103 的左正方向；不能把未知 x/z 填成示例值后启动生产定位。

车上先做无电机联调：

```bash
sudo ip addr replace 192.168.1.50/32 dev enp86s0
sudo ip route replace 192.168.1.152/32 dev enp86s0 src 192.168.1.50
ping -c 3 192.168.1.152
ls -l /dev/ttyUSB0
```

串口仍使用：

```bash
ros2 launch my_serial_py serial.launch.py \
  serial_port:=/dev/ttyUSB0 baud_rate:=115200 cmd_vel_timeout:=0.3
```

物理输出保持关闭。确认雷达、串口和真实状态反馈后，再部署 Jazzy 版导航工作区；Humble 工作区不能直接运行本仓库已经按 Jazzy 验证的二进制。
