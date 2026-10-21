# Adam_nav_2027 完整交接文档

更新日期：2026-10-21。本文覆盖车载部署、电控坐标关系、在线建图失败、LIO连续性修复和下一步地图导航交付。旧文档中的“TX=0”“尚未部署”“在线地图运行”等描述有不同时间点，**当前状态以本文与现场查询为准**。

## 1. 接手后先知道这些

- 车载主机是 **asus@192.168.1.164，SSH端口2233**，不是早期的192.168.1.145。认证沿用用户已提供的凭据，本文不公开密码。
- 本项目始终使用独立目录 `/home/asus/nav_2027_vehicle_config_164`，没有修改原车导航源码、原工作区或烧录电控。
- 当前前端是 **small_point_lio**，固定上游版本 `688d75cfa780049ae532e5100ca64f46ad8b1a93`，不是原版point_lio。
- 当前采用**雷达轴参考导航模式**，不依靠contact_angle反推出底盘姿态。完整chassis/EKF模式作为另一套入口保留，不能同时启动竞争TF。
- 本轮修复后，ASUS14包构建通过；全量 **57 tests、0 errors、0 failures、1 skipped**。另外实际small_point_lio连续性fixture、部署会话3项测试通过。
- 用户确认车辆静止后的60秒实测通过：雷达参考点最大平面变化 **0.008663m**；LIO IMU三维最大变化 **0.009214m**；姿态最大变化 **0.002649rad，约0.152°**。这是相对初始估计值的最大变化，不能当成有外部真值的定位精度。
- 角度回传已发生变化：最新直接串口只读10秒收到3328个CRC正确帧，contact_angle约 **115.624～115.646**；单位、方向、零点和机械意义仍待受控转动确认。
- **没有可用于自动导航的正式地图。**旧会话LIO严重跳变，保留地图只有1帧；不能恢复旧会话继续拼接新坐标。
- 当前运行：driver、state、foxglove。当前未运行：capture、online、bag、candidate、physical、Nav2导航。ASUS重启前的真实TX已验证，但重启后没有自动重新开启。

## 2. 仓库、分支和本机位置

GitHub：`https://github.com/kswlt/Adam_nav_2027`。

| 项目 | 值 |
|---|---|
| 当前运行源码分支 | `audit-p0-tf-costmap` |
| main | 默认首页同步使用文档；尚未合并全部运行源码 |
| 本机主开发仓库 | `C:\Users\Admin\Desktop\新导航框架\.dev\Adam_nav_2027` |
| 本机main文档工作树 | `C:\Users\Admin\Desktop\新导航框架\.dev\main_readme_20261021` |
| 原Adam参考源码 | 桌面新导航框架内`adam深北莫开源改版代码-上场版`，只读 |
| 电控原件 | 同目录两个`Season27_Regular_Sentry_*.zip`，未修改 |
| 电控解压审查目录 | `.dev/electrical_review_20261021`，不上传整份电控源码到公开仓库 |

修复检查基线为`77920aa8886ea7f0a5fb86ab5d6a48e712d07b20`，开始前已检查git status并fetch。修复提交和文档提交的最终SHA由Git日志及本次交付消息给出，避免在提交自身文件里写循环依赖的自身SHA。

LIO核心修复提交：`328ec48a0d45f2c664857ef6f398b115ef467f22`，已推送并核验远端。后续交接/会话管理提交在其后，接手请fetch最新feature，不能只停在核心修复提交。

拉取运行源码必须选对分支：

```bash
git clone --branch audit-p0-tf-costmap https://github.com/kswlt/Adam_nav_2027.git
cd Adam_nav_2027
git status
git fetch origin
git log -1 --oneline origin/audit-p0-tf-costmap
```

提交使用kswlt身份和中文说明；不要覆盖已有未提交修改，不强推重写历史。每个独立修改提交后push并用ls-remote确认实际远端SHA。用户不要求反复手工计算哈希；地图正式交付时保留已有一次性文件完整性契约。

## 3. ASUS目录、依赖与当前进程

```text
/home/asus/nav_2027_vehicle_config_164/
  remote_env_20261020.bash
  releases/20261020/                      # 旧部署，保持原样
  releases/20261021_sensor_reference/     # 本次源码、build、install、log
  runtime_20261021_sensor_reference/      # 配置、进程记录、日志、关键帧、MCAP
  deps/src/small_point_lio/               # 固定上游+既有raw-state补丁
  deps/install_lio/                      # 继续复用的small_point_lio二进制
```

雷达IP192.168.1.152；有线enp86s0使用192.168.1.50/32及既有雷达路由。无线负责SSH。不要重配成与Wi-Fi冲突的同网段路由来“修复雷达”，先检查现有route、carrier和数据。

本次启动快照（PID仅作查证，不是永久配置）：

| 组件 | 进程组长PID | 状态 |
|---|---:|---|
| driver | 6225 | 已启动 |
| state | 12675 | 已启动；含原始云质量门、输入连续性门、small_point_lio、参考状态、观测适配和子图 |
| foxglove | 12808 | 已启动，8765端口 |
| capture / online | 原PID记录仍在，实际不存活 | 未启动；旧失败会话没有恢复 |
| physical | 旧PID50244记录仍在，实际不存活 | 未启动，不发送控制数据 |

会话管理器用marker、UID、进程组核验归属；旧PID文件存在不等于进程还在运行。必须看status的running字段。

## 4. 每个终端的环境与启停命令

```bash
ssh -p 2233 asus@192.168.1.164

export NAV_BASE=/home/asus/nav_2027_vehicle_config_164
export NAV_RELEASE="$NAV_BASE/releases/20261021_sensor_reference"
export NAV_RUNTIME="$NAV_BASE/runtime_20261021_sensor_reference"
source "$NAV_BASE/remote_env_20261020.bash"
source "$NAV_RELEASE/install/local_setup.bash"
export ROS_DOMAIN_ID=97
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

remote_env会加载Jazzy、现有Livox驱动、small_point_lio、依赖库和兼容PAL overlay；本次install必须最后加载。不要混入原车工作区。

查询和正常启动：

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" status state
python3 "$NAV_RUNTIME/vehicle_session.py" start driver
python3 "$NAV_RUNTIME/vehicle_session.py" start state
python3 "$NAV_RUNTIME/vehicle_session.py" start foxglove
```

如果已运行，start返回已有实例，不重复创建。停止只影响本项目标记的进程组：

```bash
for component in physical bag online capture candidate foxglove state driver; do
  python3 "$NAV_RUNTIME/vehicle_session.py" stop "$component"
done
```

修复后的stop会等待最多8秒，确认旧实例退出；超时返回错误，不能把仍在退出的实例说成已重启。不使用pkill/killall停止其他导航。

## 5. 当前架构和14个包

```text
MID-360官方Livox驱动
  ├─ /livox/lidar → 原始几何/时间质量门 → guarded_points ─┐
  └─ /livox/imu ────────────────────────────────────────┤
                               双输入连续性门 lio_input_guard
                                      ↓ 原消息/原时间戳
                                small_point_lio
                     ┌────────────────┴─────────────────┐
               原始IMU 6DoF位姿                    去畸变odom云
                     ↓ 位姿连续性检查                   ↓
               sensor_reference                    观测适配/滚动子图
              TF、平面参考状态                          ↓
                     └──────────────────────────── 关键帧档案
                                                       ↓
                                  在线KISS/GICP回环 + GTSAM历史地图

正式地图 + 冻结地图定位 → MapOdomManager → map→odom
正式地图 + Nav2/TDT → 平移安全命令 → 雷达水平系xy → 私有Adam适配器
                                                        ↓ 可选开启
                                                    my_serial_py
                                                        ↓
                                           电控机械角转换/轮子/云台
```

| 包 | 主要职责 |
|---|---|
| my_serial_py | 原协议编解码、真实串口收发与连接状态 |
| pb_rm_interfaces | 原裁判/业务ROS接口兼容 |
| rm_nav_interfaces | 观测、轨迹、任务等框架接口 |
| rm_nav_frames | 标定与坐标关系公共逻辑 |
| rm_nav_sensors | MID360质量门、LIO观测适配、传感器健康 |
| rm_nav_localization | 滚动子图、冻结地图定位、map/odom管理 |
| rm_nav_registration | small_gicp/KISS等真实配准封装 |
| rm_nav_mapping | 关键帧录制、在线/离线回环、图优化与地图重建 |
| rm_nav_perception | 环境/地形等感知支持，按实际profile启用 |
| rm_nav_planning | 轨迹与规划支持 |
| rm_nav_control | TDT/Nav2控制器、Omni/Yaw相关控制与碰撞验证 |
| rm_nav_hardware | ros2_control硬件接口与反馈/命令契约 |
| rm_nav_bringup | launch、配置、监管和现场工具入口 |
| rm_nav_sim | 隔离的软件/理想底盘验收支持 |

软件测试通过不代表每个包都已经接到真实硬件闭环。当前雷达参考兼容输出没有启动ros2_control物理接口。

## 6. 坐标、安装和电控yaw的准确含义

用户最新确认几何：车体外轮廓50×50cm；yaw轴位于底盘中心左侧16cm；雷达与该轴同轴、随轴转动；底盘顶面离地20cm，雷达测量原点再高5cm，即离地约25cm。早期左偏8cm/离地20cm配置已被此说明替代。

MID360内部IMU→LiDAR采用当前工厂配置平移`[-0.011,-0.02329,0.04412]m`和单位旋转；驱动使用identity外参，不能再重复叠加车体安装位移。这是采用的配置来源，不等于现场已独立验证所有安装轴向。

| frame/变换 | 定义和发布者 |
|---|---|
| odom | 本次LIO初始化参考；重启状态链会产生新估计坐标，不与旧会话无缝等同 |
| odom→front_mid360_imu | sensor_reference从已通过检查的原始LIO位姿发布 |
| front_mid360_imu→front_mid360 | 工厂内部外参，static broadcaster |
| odom→nav_reference | 雷达轴的平面位置；z=0、朝向与odom一致；不是底盘车头 |
| odom→sensor_yaw_reference | 同一平面点，水平朝向随雷达LIO yaw变化 |
| mapping_odom | 建图后端的优化坐标；不自动等于map |
| map→odom | 仅真实地图定位后的MapOdomManager发布；当前建图状态不存在 |

small_point_lio实际publish_tf=false；它仍可能创建/tf发布者，不能只看publisher列表就断言它实际在发TF。

当前不发布冒充真实的chassis/base_footprint/base_link旋转。现在有非零contact_angle，也不能在单位、符号、零点和采样时刻未确认前直接打开机械TF。

原Adam的gimbal_link_fake是“位置随车走、方向抵消云台yaw”的虚拟导航参考点。新模式nav_reference沿用这个控制思路。

速度链：固定参考方向xy → 上位机按LIO yaw转到雷达水平朝向 → my_serial_py按原协议给xy各取负 → 云台板串口 → CAN → 底盘板按真实大yaw编码器角转到底盘内部轴 → 四轮逆解。

电控不需要知道“底盘朝南”，只需知道底盘与云台差多少角度。上位机处理世界/雷达关系，电控处理云台/底盘关系；不要在上位机再按机械角重复旋转。

**协议朝向与速度作用点是不同问题。**雷达轴左偏16cm，底盘自旋会让雷达轴绕圈；中心速度与轴速度的偏心补偿尚未确认，不能把只完成旋转矩阵说成完整补偿。

## 7. 电控通信现状与禁止混淆的字段

实际串口：

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
→ /dev/ttyUSB0
115200；RX26字节，TX54字节；原CRC和字段顺序保留
```

上传电控源码中云台板曾把contact_angle写死0；最新只读采集已收到非零值，说明不能继续用旧ZIP推断当前上传字段仍恒0，但新固件具体改动未审查。

10秒只读实测：86528字节、3328有效帧、CRC错误0、TX字节0，contact_angle最小115.6239319、最大115.6459045。原始rx.bin及report在runtime的`angle_rx_1792577348812492818`，仓库保存JSON摘要。

下一步确认角度必须用真实机械动作：先确认对齐位置的数值，再底盘不动转云台约+30°、回正、反向，核对符号、周期、单位和零点。没有执行本轮动作测试。

**串口yaw语义有冲突：**旧上位机名为yaw_degrees并写入两个float；上传电控把第3个叫rotate、第4个叫yaw_speed。实际第4个经CAN到planning_spd，在Navigation、无自瞄跟踪、running_state=5时按度/秒使用；默认导航分支扫描80°/s。running_state=3时发yawfloat=0不保证云台停止。不要把LIO yaw填入这两个字段。

电控ZIP另有待核实问题：Navigation_online倒计时没有用于清零导航缓存xy；底盘Navigation分支W_set=0。是否与当前烧录版本一致待确认。没有修改电控。

重启前真实TX曾验证2.013秒100个54字节成功write，约50Hz；那时xy=0、yawfloat=0、stance=3。这是内核写入成功，不是固件动作反馈。**当前physical关闭**。

显式启用真实发送的命令（会打开设备，不属于只读检查）：

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" stop candidate
python3 "$NAV_RUNTIME/vehicle_session.py" start physical
python3 "$NAV_RUNTIME/vehicle_session.py" status physical
ros2 topic echo /hardware/legacy_adam/serial_healthy std_msgs/msg/Bool --once
```

关闭：`python3 "$NAV_RUNTIME/vehicle_session.py" stop physical`。不得同时运行candidate和physical或第二个串口节点。关闭串口不等于下位机自旋停止，当前源码断流缓存也不能当成已验证watchdog。

## 8. 本次LIO故障、修复和验证细节

### 故障证据

旧会话`session_1792568087261746637_4085448541`：第0帧位置约(0.003,0.003)，4774.399秒后第1帧估计(-4626.836,-1126.821)m，跳4762.076m；随后约0.1秒估计移动数百米。16帧原件保留。

在线后端锁存`LIO adjacent jump exceeds online mapping limits`；离线优化返回1、`Adjacent seed jump too large`。地图质量门正确拒绝，不是提高优化跳变阈值可以解决的问题。

检查固定上游small_point_lio：滤波预测直接用timestamp-last_timestamp做dt；没有长断流恢复的上限。原传感器参考节点只检查frame、时间新鲜度和协方差，没有物理运动连续性门。**确认的是这两个软件缺口与可复现失效机制；导致旧记录原始断流的具体硬件/网络原因未被旧数据证明。**不能把补门控说成已查明全部硬件根因。

### 修复范围

新增`src/rm_nav_bringup/scripts/lio_input_guard.py`和`lio_continuity.py`，由sensor_reference_state.launch接入：

- 原始质量门后的点云与原始IMU先经过双输入门，再到`/lio/input/cloud`、`/lio/input/imu`。
- 只转发原消息，不改时间戳、frame、IMU单位，不插值或生成虚假数据。
- 两种输入都有效后才开始；初始发现阶段过期DDS排队消息丢弃，不用于初始化。
- 运行中任一流到达/源时间间隔超过默认0.5秒、时间倒退或源数据过期，锁存失败；停止给旧滤波器继续输入，不无限堆积IMU。
- 两路DDS回调不同步，不能直接把“最新IMU时间减上个扫描起始时间”当故障；启动同步与运行期间逐流年龄/间隔分别检查。
- 只能明确重启state进程重新初始化，不自动跳回零点并混入旧地图。

sensor_reference增加位姿连续性检查：默认相邻源时间不超过1秒、传感器速度6m/s、角速度12rad/s，另有小数值容差；这些是拒绝明显失真的上限，不是性能验收指标，也没有放宽旧建图门限。异常位姿不再进入导航里程计/TF，健康关闭并清除命令。

本轮没有改写small_point_lio滤波算法、没有换前端、没有覆盖deps中的既有上游修改。新输入门目前明确接在雷达参考入口；直接运行裸small_point_lio或其他旧launch不自动获得该保护。

### 测试与真实静止结果

- ASUS全14包build通过；colcon test / test-result：57测试、0错误、0失败、1可选libscrc跳过。
- 连续性单测覆盖双输入等待、正常流、源/到达断流、时间倒退/过期、启动排队、异步扫描、公里级位移、姿态跳变和故障锁存。
- 真实small_point_lio隔离fixture输出46组pose/cloud，单独断点云后恢复两路，旧状态不继续输出；不是用假LIO替代滤波器。
- 状态ROS测试确认公里级异常不进入导航odom，后续正常数据不会自动解锁；串口PTY回归通过，协议格式未修改。
- 旧测试曾瞬间跳变yaw，现改为有界转动并留出输出稳定时间；没有提高生产门限来迁就不物理的测试输入。
- 管理器3项测试通过，停止等待、立即重启会得到新进程；仍有后台Popen生命周期ResourceWarning，不代表测试失败。
- 用户确认静止后60秒实测：566组LIO位姿/去畸变云、576条参考odom、299个子图、60个预览；原始云584条。计数差不是严格丢帧率。
- LIO三维估计最大变化0.0092135m，姿态0.00264874rad；平面参考点变化0.00866333m。LIO源消息年龄中位7.41ms、p95约10.12ms，均为ASUS内部时钟，不是Foxglove端到端延迟。
- 输入、原始云、参考状态、子图最终健康均true，TF父关系符合设计；Python MessageInfo没有publisher_gid，未伪造逐消息authority验证。

测试证据：`docs/evidence/lio_continuity_static60_20261021.json`和`lio_fix_colcon_test_result_20261021.txt`。静止探针未同步录成新的MCAP；不要把此前MCAP说成本次60秒的原始记录。下一次动态验收须同时录bag。

首次增量build因跨机文件时间先后未重新配置CMake，出现新增模块未安装的测试失败；强制配置和更新本次文件时间后修复。首次输入门把不同步回调误判为跨传感器时间差，已改为逐流检查并补单测。失败日志保留，最终57项通过覆盖修正后代码。

## 9. Foxglove怎么查看

连接`ws://192.168.1.164:8765`。当前只有实时链，online尚未启动。

| 显示层/话题 | 参考系 | 意义 |
|---|---|---|
| /livox/lidar | 通过TF放到odom查看 | 原始当次扫描，不是地图 |
| /lio/deskewed_odom_cloud | odom | small_point_lio当前去畸变配准扫描 |
| /localization/odom_submap | odom | 有界滚动子图，不是永久全图 |
| /visualization/live_map_preview | odom | 历史累积预览，不修正历史回环 |
| /visualization/sensor_reference_path | odom | 雷达轴参考轨迹，不是底盘中心轨迹 |
| /mapping/optimized_map_preview | mapping_odom | online启动后才有，回环优化的历史地图预览 |
| /mapping/optimized_path、loop_graph | mapping_odom | 优化轨迹、关键帧/回环边 |
| /lio/input_healthy、input_reason | 不限 | 双输入连续性与锁存原因 |
| /state/sensor_reference_healthy、reason | 不限 | 当前参考定位链是否可用 |
| /contact_angle | 不限 | 真实串口节点运行时的回传；当前未启动该节点 |

布局：`src/rm_nav_bringup/config/foxglove/sensor_reference_mapping.json`与`mapping_online.json`。同一面板不要未经转换混合odom与mapping_odom。

颜色常为高度/强度，红色不默认等于障碍。`/visualization/map_cloud.header`等待通常是尚未加载冻结地图，不代表原始雷达没数据。日常只开一个大点云，优先preview，关闭长时间点云衰减累积；预览1Hz不等于LIO只有1Hz。

## 10. 重新建图：必须新建会话

当前稳定状态链可继续使用；旧失败session只读保留，不要把state重启后的新odom写回它。

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" start bag
python3 "$NAV_RUNTIME/vehicle_session.py" start capture
ros2 topic echo /mapping/recorder_status std_msgs/msg/String --once
```

bag默认最多180秒，保存runtime/bags。记下recorder_status给出的**新session完整路径**，然后：

```bash
export NAV_SESSION=/这里填写刚生成的新session完整路径
python3 "$NAV_RUNTIME/vehicle_session.py" start online "$NAV_SESSION"
ros2 topic echo /mapping/online_status std_msgs/msg/String --once
```

先遥控低速直线1m、返回，再横移和小角度旋转；每次动作留静止段并记录测量距离/角度。观察重影、轨迹和reason；异常立即停止本次采集，不放宽漂移/跳变门限。

完成后停车：

```bash
python3 "$NAV_RUNTIME/vehicle_session.py" stop online
python3 "$NAV_RUNTIME/vehicle_session.py" stop capture
python3 "$NAV_RUNTIME/vehicle_session.py" stop bag
```

在线图单段最多500关键帧，地图最多500000体素，候选预算有界；超限报fault保留有效快照。当前nav_reference yaw固定，只转云台不平移不能靠虚拟yaw阈值触发新关键帧，这仍是待修项；实时预览更新不等于关键帧一直保存。

## 11. 地图导出与导航剩余距离

旧检查包已下载本机`output/map_export_20261021/retained_review_20261021`，ASUS位于runtime/map_exports同名目录。含PCD、occupancy.pgm、map.yaml、terrain_reference和CDR快照，但只有1帧2806点。PGM没有自由空间证据，明确navigation_ready=false；不可用于车辆导航。

新有效session停止录制后，可运行已有离线真实后端：

```bash
cd "$NAV_RELEASE"
ros2 run rm_nav_mapping offline_graph_optimizer \
  "$NAV_SESSION" "$NAV_RUNTIME/map_exports/请填写全新优化目录" --enable-loops
```

若拒绝，先看相邻约束/几何/跳变原因，不能插入假边。输出成功后使用create_map_bundle生成draft；需要把清单放在能正确引用同目录地图文件的位置，不能造跨Bundle的逃逸路径。

本次用户选择**本地场地坐标**。现有正式发布器及启动器仍强制官方对齐、verified官方自由空间mask；因此下一步需要明确实现local_site的发布/启动契约：保持source、坐标定义、地面高度、自由空间证据、地图版本和定位验证，独立标明非官方。不能简单把official_alignment_applied设true。

点云端点只能说明击中了什么，不能说明没点的地方都可通行。占据图需有合理射线/地面观测或人工审核的自由空间来源；当前terrain minimum_observed_z也不是可通行地形真值。

拿到符合契约的实际导航地图后才启动：

```bash
ros2 launch rm_nav_bringup sensor_reference_navigation.launch.py \
  map_bundle:=/真实可导航Bundle/map_bundle.json
```

Foxglove在map中给/initialpose定位提示，选雷达轴位置；真实匹配通过后再通过/nav/submit_goal提交任务。当前正式Bundle缺失，该命令现在会拒绝，不是已经完成的一键上车入口。

## 12. 开发构建、测试与故障恢复

ASUS加载第4节环境后：

```bash
cd "$NAV_RELEASE"
colcon build --executor sequential --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose

# 单独的TEST_ONLY测试域；不得把fixture接入Domain97。
ROS_DOMAIN_ID=209 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 tools/smoke_small_point_lio.py --with-input-guard
```

普通源文件更改不必每次重跑全套；有代码、安装或失败修正时执行相关测试及必要全量回归。本轮最终构建/测试日志在runtime的lio_fix_build.log、lio_fix_test.log、lio_fix_test_result.log。failed1/failed2记录保留；不是删掉失败后宣称一次通过。

现场查询：

```bash
ros2 topic echo /lio/input_reason std_msgs/msg/String --once
ros2 topic echo /state/sensor_reference_reason std_msgs/msg/String --once
ros2 topic echo /sensors/front_mid360/cloud_reason std_msgs/msg/String --once
ros2 topic hz /lio/sensor_odometry
ros2 run tf2_ros tf2_echo odom front_mid360
```

发现等待时，显式提供消息类型，或`ros2 node list --no-daemon --spin-time 3`。先核对Domain和进程存活，再判断网络/驱动；重启后/tmp探针不存在是正常现象，不能把零样本说成电控回传0。

输入锁存故障恢复：停止physical和任务输出，停车；停止bag/online/capture并保留数据；排除断流/时间/硬件问题；stop state等待退出，再start state。必要时重启本工程foxglove以清除新旧odom累积。确认健康与静止数据后，另建session录制。不能自动解除故障后续写旧地图。

## 13. 后续工作顺序和仍未验证的项目

1. 取得当前烧录电控版本/差异，完成contact_angle对齐、±30°方向和单位验证；目前只知道收到非零值。
2. 同步录MCAP做直线、横移、旋转与返回数据；检查新连续性门在真实运动下的边界，不能仅凭静止通过上车验收。
3. 修复纯云台转动的关键帧触发；保持有界录制和原始数据，不更换LIO前端。
4. 新会话真实回环与地图几何检查，随后实现local_site MapBundle交付、地面/自由空间来源和冻结地图定位。
5. 核对电控导航自旋W_set、断流停车缓存、遥控接管、命令超时及16cm偏心速度作用点。接收心跳不是轮速或停车反馈。
6. 最后在受控场地下验证真实xy方向、目标到达、障碍停止、定位失效停止、串口断流与接管，再决定持续自动导航。

没有完成：真实完整底盘TF标定、角度方向/零点/时刻、轮速/停车反馈、偏心自旋补偿、动态定位真值、正式本地场地Bundle、整车自动行驶和长期稳定性。不得把ROS/PTY/理想底盘结果等同实车验收。

本轮到交接时未启动新的运动指令、未烧录电控、未合并功能源码到main、未删除原始MCAP或失败档案。当前driver/state/Foxglove可继续观察，正式导航仍未启用。
