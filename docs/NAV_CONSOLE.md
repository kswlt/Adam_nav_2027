# 中文终端菜单：建图、导航、日志和Foxglove

菜单在ASUS的SSH终端里运行，退出菜单不退出后台组件。它调用既有 `vehicle_session.py`，仅管理本项目带标记的进程，不覆盖或停止原车工作区。源码在开发分支 `audit-p0-tf-costmap`；ASUS当前离线，这次尚未部署。

## 启动菜单

先部署并构建最新release。下面只是把三个控制台文件装入运行目录，不代替完整代码部署：

```bash
ssh -p 2233 asus@192.168.1.164
export NAV_BASE=/home/asus/nav_2027_vehicle_config_164
export NAV_RELEASE="$NAV_BASE/releases/20261021_sensor_reference"
export NAV_RUNTIME="$NAV_BASE/runtime_20261021_sensor_reference"
cd "$NAV_RELEASE"
install -m 755 tools/deployment/20261021/nav_console.py "$NAV_RUNTIME/nav_console.py"
install -m 755 tools/deployment/20261021/vehicle_session.py "$NAV_RUNTIME/vehicle_session.py"
install -m 755 tools/deployment/20261021/vehicle_component.bash "$NAV_RUNTIME/vehicle_component.bash"
python3 "$NAV_RUNTIME/nav_console.py"
```

控制台本身只用Python标准库，不需要安装网页服务。消息/服务命令自动加载独立release环境并使用ROS Domain97。

## 一次建图怎么操作

1. 选 **2**：关闭本项目的物理输出和导航，启动driver、state、foxglove、capture及限时180秒MCAP录制。关键帧采集不会随180秒录包自动结束。
2. 菜单等待本次新创建的唯一关键帧会话，最长20秒，然后请求启动在线优化。若采集已经运行或没确认唯一新会话，菜单会提示用 **3** 明确选会话，不自动挑一个旧目录。
3. 选 **5**，输入 `state` / `capture` / `online`，终端实时打印组件日志。Ctrl+C返回菜单，后台继续运行。
4. 选 **6→1** 看累计地图状态，**6→2** 看关键帧、回环与故障。开Foxglove观察累计地图。
5. 建完后停车，先用 **e** 导出当前在线地图检查包；输入新的绝对输出目录及实际mapping_odom坐标中的障碍物高度范围。导出时online仍需运行。输出含PCD、检查用PGM/YAML、terrain及draft清单，不自动变成正式地图。
6. 导出完再选 **s** 停止本项目全部组件。原始录包和关键帧仍在磁盘。

菜单2不是软件/硬件验收通过提示：如果设备、串口或LIO没启动，组件可能随后退出，选1/5/6查实际状态。旧参考点会话不应混入当前拟合debug-v3会话。

## 建完后导航怎么操作

建图检查包仍需做自由空间审核、地图交付和定位验证。不能把 `draft_mapping_review` 直接填入导航入口；菜单复用已有启动器继续检查正式MapBundle合同。

1. 选 **4**，输入可导航 `map_bundle.json` 的绝对路径；当前房间调试选 `local_site`，比赛官方坐标选 `official`。菜单停止capture/online/录包及真实串口，然后启动状态链、Foxglove、导航和无设备候选适配器。
2. Foxglove在map参考系给 `/initialpose` 发布初值；初值位置对应导航轴心参考，不是偏心LiDAR中心。等待真实地图匹配健康。
3. 选 **6→6** 查看任务状态；选 **7** 输入实际地图中可行的 `x y` 目标。它调用 `/nav/submit_goal`，遵守现有任务监管器许可。
4. 默认是影子导航。**8** 才单独开启真实串口输出，**9** 关闭它。若8开启后已有有效导航任务，电控可能收到运动指令。不能把“退出菜单”当成停车；用9或s停止本项目输出，现场遥控器接管仍独立存在。

停止本项目的平移/串口不代表电控自行扫描或自旋也停止，沿用现有协议语义。

## 终端到底看什么

菜单 **5** 是进程日志，菜单 **6** 是ROS消息，二者不同。组件启动后stdout/stderr保存到运行目录；新增组件使用Python无缓冲输出，tail会连续显示日志。静默进程不一定故障，需要看话题状态。

| 菜单6编号 | 内容 |
|---|---|
| 1 | 累计点数、体素分辨率、输入是否过期 |
| 2 | 关键帧、回环、地图后端状态 |
| 3 | 传感器参考定位的实际健康原因 |
| 4 | 速度转换、LIO朝向时效、拟合标识 |
| 5 | 串口适配器是否放行 |
| 6 | 当前导航任务状态 |
| 7 | 原始LIO里程计 |
| 8 | 电控角度及主机接收时间，不是MCU采样时间 |

手工命令也可用，新SSH终端先加载环境：

```bash
source "$NAV_BASE/remote_env_20261020.bash"
source "$NAV_RELEASE/install/local_setup.bash"
export ROS_DOMAIN_ID=97
ros2 topic echo /visualization/live_map_status --field data
ros2 topic echo /mapping/online_status --field data
ros2 topic hz /livox/lidar
```

不要直接连续echo整个PointCloud2的data数组，终端会刷大量字节，无法有效观察地图。

## Foxglove这样配置

连接 `ws://192.168.1.164:8765`；菜单 **f** 也会打印说明。布局JSON在release的 `src/rm_nav_bringup/config/foxglove/`，本机仓库同一相对路径也有，导入到桌面Foxglove即可。

| 目的 | 导入布局 | 3D参考系 | 主要点云 |
|---|---|---|---|
| 快速累计覆盖检查 | `sensor_reference_mapping.json` | `odom` | `/visualization/live_map_preview` |
| 在线回环和历史修正 | `mapping_online.json` | `mapping_odom` | `/mapping/optimized_map_preview` |
| 地图定位和导航 | `sensor_reference_navigation.json` | `map` | `/map`、局部Costmap、规划轨迹、导航参考点TF |

累计图每条消息已经含全局预览，客户端无需长时间堆多帧；同一时刻优先只开一种大点云。TF在3D的Transforms中勾选odom、nav_reference、sensor_yaw_reference、front_mid360；小箭头是坐标轴，不是规划路径。菜单6→1显示 `voxel_m`，变大表示预览为保留覆盖而降低分辨率，不是旧地图被删除。

新的雷达参考导航布局已勾选 `nav_reference` 和 `sensor_yaw_reference`，并显示任务/定位/速度状态；不应期待该模式发布真实chassis/base_footprint TF。没有已接受的地图匹配结果时，map相关显示和路径可能还不可用。

更详细的点云/内存解释见 [累计点云指南](ACCUMULATED_MAP_VIEW.md)。

## 本轮验证与边界

基线 `0d37444`。`python3 tools/test_nav_console.py` 3项通过：建图不自动开启串口；影子导航准确保留含空格的地图路径且不开启串口；停止操作只经过现有受标记管理器。Bash语法及Python编译检查通过。

本轮没有改变ROS节点算法，未重复上轮64项ROS回归；没有ASUS/串口/菜单整流程实机测试。按菜单选择地图后，地图是否可用仍由真实launch合同和定位结果决定。原有未提交的 `tools/smoke_local_site_navigation.py` 修改保留，未纳入本轮提交。中文提交推送开发分支，未合并main。
