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

Livox 驱动的平移字段单位为毫米且必须是整数。本框架保留原始 LiDAR 坐标，驱动必须使用恒等变换：

```json
"x": 0,
"y": 0,
"z": 0
```

车体安装位移 x=0、y=0.08、z=0.20 m 只在安装 TF 中应用一次，不能再写进驱动点云变换。Point-LIO 显式读取 extrinsic_T/extrinsic_R；工厂内部参数必须核对方向并写入 LIO 标定配置，驱动不会自动替 LIO 填写。当前尚未完成本车内部外参与 TF 的联合验收。

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
source /home/asus/nav_2027_vehicle_config_164/workspace/tools/vehicle_164_env.bash
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
ros2 launch rm_nav_bringup mid360_diagnostic.launch.py \
  driver_config:=/home/asus/nav_2027_vehicle_config_164/workspace/config/hardware/mid360_car_164.json \
  lio_params_file:=/home/asus/nav_2027_vehicle_config_164/deps/src/small_point_lio/config/mid360.yaml
```

启动配置位于隔离源码：

```text
/home/asus/nav_2027_vehicle_config_164/workspace/config/hardware/mid360_car_164.json
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

当前不能宣称已能实车导航：网口 carrier=0，实时点云、LIO 和 TF 联合验收未完成。新框架实际启动 small_point_lio，原版 point_lio 只作参考，不能代替该依赖。

## 9. 物理控制安全边界

- 轮速反馈和停车反馈按用户要求暂缓接入。
- `enable_physical_output` 默认保持 `false`。
- 没有经过急停、watchdog、零速报文和低速空旷场地检查前，不启用物理输出。
- 不使用虚拟健康话题冒充实测反馈。
- 不停止、不覆盖原有导航进程。

## 10. 下一步

1. 接通 MID-360 网线，确认 `carrier=1`、能 ping `192.168.1.152`。
2. 使用保留原始坐标的 `x=0,y=0,z=0` JSON 启动驱动，确认 `/livox/lidar` 和 `/livox/imu`。
3. 启动 Point-LIO，检查点云、IMU、`odom` 和 TF 时间连续性。
4. 在独立 ROS Domain 中启动 Costmap/Nav2，先只观察 `/nav/cmd_vel_safe`。
5. 串口协议抓包确认零速、平移、yaw 角字段。
6. 低速、空旷、人工急停在场后，再进行首次实际移动。


## 11. 本轮部署修复与限制

- 新增 `tools/vehicle_164_env.bash`，加载 Jazzy、独立依赖、small_point_lio 和导航工作区，使用 ROS_DOMAIN_ID=97。
- 在独立工作区执行 `bash tools/vehicle_164_build.bash`，构建固定版本 small_point_lio，再构建和测试导航包。原版 driver/point_lio 副本不加入导航包测试列表。
- 离线优化器直接依赖 small_gicp 导入目标，不再使用全局 `-I` 掩盖缺失依赖。
- 上面的 diagnostic 启动仅用于传感器诊断，不发布车体 TF、不启动串口、不代表生产定位通过。指定的上游 YAML 内部外参仍需核对；请先用 `ls` 确认实际文件名。
- 本车 `x/y/z` 来自用户提供尺寸，不标记为实测 verified CalibrationBundle。
- 系统 ROS 依赖安装会修改 /opt/ros 环境；隔离的是源码、构建产物和运行 Domain，不能声称系统依赖完全隔离。
- Omni 适配器、真实反馈及 HardwareInterface 的运行验收尚未完成，编译通过不能证明可实车导航。

## 12. Omni 适配器接续

已完成空反馈解引用、命令/odom/就绪超时、最终向量限幅和零速/yaw 重置修复。
无轮速模式为纯前馈，仍保留真实 odom 与 drive_ready 要求；不是闭环 PID 已调参。
车载 13 包构建通过，41 项测试 0 错误、0 失败、1 跳过；新增回归在 Domain 199、
`/test_only/omni/*` 下执行，未打开串口。见 [Omni 验收](omni_adapter_acceptance.md)。
当前 my_serial_py 保留旧 yaw 角，平移超时不能保证旋转停止，必须核验下位机停止语义。

网络检查：到 192.168.1.152 的 /32 路由走 enp86s0、源地址 192.168.1.50；
Wi-Fi SSH 保持正常。ethtool 显示 Link detected: no、速度未知，网卡收发计数为 0。
重新协商未恢复；未修改 Wi-Fi、持久 IP 配置或重载网卡驱动。雷达实际 IP 尚不能确认，
需要先恢复物理链路。不能只凭雷达通电判定以太网正常。
