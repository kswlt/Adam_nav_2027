# 车载电脑 192.168.1.164 实车配置

已确认车载电脑：

| 项目 | 实测值 |
| --- | --- |
| SSH | `asus@192.168.1.164:2233` |
| ROS 工作区 | 实测 /opt/ros/jazzy；新工作区独立部署，原工作区保持原状 |
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

MID-360 内部 LiDAR-IMU 工厂参数需显式写入 Point-LIO 的 extrinsic_T/extrinsic_R，并核对方向和坐标轴。驱动 JSON 使用恒等变换保留原始点云坐标，车体安装位置仅在 TF 中应用一次。本车外参与实时 TF 尚未联合验收。

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

物理输出保持关闭。独立 Jazzy 工作区已经部署；环境加载、构建和当前阻塞见 HANDOFF_VEHICLE_164.md。
