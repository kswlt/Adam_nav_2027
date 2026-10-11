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
| 雷达安装 | 正装，车辆中心左侧 0.08 m，纵向 0 m，高度 0.20 m |

已生成：

- `config/hardware/mid360_car_164.json`：Livox host/radar 网络配置。
- `config/hardware/vehicle_car_164.yaml`：车载硬件 profile。

当前 profile 仍保留硬件安全门。已采用用户确认的安装尺寸：`x=0 m`、`y=+0.08 m`、`z=0.20 m`，其中 `y` 采用 REP-103 左正方向。轮速反馈和停车反馈暂按用户要求延期接入，但延期不等于伪造反馈。

MID-360 的 LiDAR-IMU 内部外参由设备/驱动提供，配置中的 `imu_extrinsic_source` 标记为 `mid360_factory_calibration`。这解决的是雷达内部 LiDAR 与 IMU 的关系；雷达整体相对车体的安装变换仍由本车的 `x/y/z` 和姿态参数描述，不能用设备内部标定替代。

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
