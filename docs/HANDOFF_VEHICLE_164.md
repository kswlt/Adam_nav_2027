# Adam_nav_2027 实车交接文档（192.168.1.164）

更新时间：2026-10-11

## 1. 交接范围

本次部署全部位于车载电脑的独立目录：

```text
/home/asus/nav_2027_vehicle_config_164/
```

没有修改以下原有工作区、启动文件或运行中的进程：

```text
/home/asus/桌面/ros_ws
/home/asus/sentry_code/Adam_sentry_attack26/navigation/ros_ws
/home/asus/SCURM_SentryNavigation
```

当前新工作区为：

```text
/home/asus/nav_2027_vehicle_config_164/workspace
```

## 2. 实车已确认参数

| 项目 | 当前值 |
| --- | --- |
| 车体 | 全向轮、单 yaw |
| ROS | ROS 2 Jazzy |
| 车载 Wi-Fi | `192.168.1.164` |
| 雷达网卡 | `enp86s0`，配置地址 `192.168.1.50/32` |
| MID-360 | `192.168.1.152` |
| 雷达安装 | 正装；车体中心左侧 80 mm；纵向 0 mm；高度 200 mm |
| 串口 | `/dev/ttyUSB0`，Silicon Labs CP2102，115200 baud |
| 串口协议 | 原版 `my_serial_py`，54 字节 TX、26 字节 RX、CRC |

Livox 驱动的平移外参字段单位是毫米，必须使用：

```json
"x": 0,
"y": 80,
"z": 200
```

雷达内部 LiDAR-IMU 标定由 MID-360 工厂参数和驱动使用；上述 JSON 是雷达整体相对车体的安装变换。

## 3. 已完成内容

- 已从 GitHub feature 分支克隆源码到隔离工作区。
- 已在车上单独编译新框架及其依赖：Nav2、robot_localization、ros2_control、GTSAM、small_gicp、KISS-Matcher/ROBIN。
- 已复制并单独编译原版 `livox_ros_driver2` 和 `point_lio` 源码。
- 已复制原版 `my_serial_py` 协议实现到隔离工作区，原工作区未改动。
- 全量 CTest：40 项测试，0 失败，1 跳过。
- `rm_nav_control` 的 Omni PID 与命令合成测试通过。
- `rm_nav_registration` 的 small_gicp/KISS 测试通过。
- `rm_nav_mapping`、`rm_nav_localization`、`rm_nav_planning`、`rm_nav_bringup` 测试通过。

## 4. 当前目录

```text
/home/asus/nav_2027_vehicle_config_164/
├── README.txt
├── vehicle_car_164.md
├── config/hardware/mid360_car_164.json
├── config/hardware/vehicle_car_164.yaml
├── deps/
├── workspace/
├── isolated_build.log
└── isolated_test.log
```

## 5. 环境加载

```bash
source /opt/ros/jazzy/setup.bash
source /home/asus/nav_2027_vehicle_config_164/workspace/install/setup.bash
export CMAKE_PREFIX_PATH=/home/asus/nav_2027_vehicle_config_164/deps/install:/opt/ros/jazzy/opt/small_gicp_vendor:${CMAKE_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/home/asus/nav_2027_vehicle_config_164/deps/install/lib:/opt/ros/jazzy/opt/small_gicp_vendor/lib:${LD_LIBRARY_PATH:-}
```

## 6. 雷达启动前检查

```bash
ip -brief addr show enp86s0
cat /sys/class/net/enp86s0/carrier
ping -c 3 192.168.1.152
```

当前最后一次检查显示 `enp86s0` 为 `NO-CARRIER`，所以 MID-360 尚未实际连通。临时测试命令如下，未写入持久网络配置：

```bash
sudo ip link set enp86s0 up
sudo ip addr replace 192.168.1.50/32 dev enp86s0
sudo ip route replace 192.168.1.152/32 dev enp86s0 src 192.168.1.50
```

雷达网线或交换机链路恢复后，重新执行 ping，再启动：

```bash
export ROS_DOMAIN_ID=97
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

启动配置位于隔离源码：

```text
/home/asus/nav_2027_vehicle_config_164/workspace/src/livox_ros_driver2/config/MID360_config.json
```

## 7. 串口启动

```bash
export ROS_DOMAIN_ID=97
ros2 launch my_serial_py serial.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  cmd_vel_timeout:=0.3
```

串口节点默认超时归零平移命令，且只由隔离工作区的 `my_serial_py` 进程打开 `/dev/ttyUSB0`。同一时刻不要启动原工作区的串口节点。

## 8. 导航启动顺序

雷达链路和真实 LIO/TF 验证通过后，顺序为：

1. MID-360 驱动。
2. Point-LIO。
3. `rm_nav` 的 TF、定位健康和 `/odom` 链。
4. `/scan` 与 Costmap。
5. Nav2 MPPI 基线。
6. 串口节点。
7. 经过人工确认后才允许物理输出。

当前不能直接宣称“已经可以导航控制车辆”，因为最后一次雷达启动因错误的 JSON 数值类型退出，且网口没有 carrier；修正后的毫米 JSON 尚待雷达重新接通后验证。

## 9. 物理控制安全边界

- 轮速反馈和停车反馈按用户要求暂缓接入。
- `enable_physical_output` 默认保持 `false`。
- 没有经过急停、watchdog、零速报文和低速空旷场地检查前，不启用物理输出。
- 不使用虚拟健康话题冒充实测反馈。
- 不停止、不覆盖原有导航进程。

## 10. 下一步

1. 接通 MID-360 网线，确认 `carrier=1`、能 ping `192.168.1.152`。
2. 使用修正后的 `x=0,y=80,z=200` JSON 启动驱动，确认 `/livox/lidar` 和 `/livox/imu`。
3. 启动 Point-LIO，检查点云、IMU、`odom` 和 TF 时间连续性。
4. 在独立 ROS Domain 中启动 Costmap/Nav2，先只观察 `/nav/cmd_vel_safe`。
5. 串口协议抓包确认零速、平移、yaw 角字段。
6. 低速、空旷、人工急停在场后，再进行首次实际移动。

