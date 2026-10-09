# 底盘状态桥与主 LIO 观测子地图

2026-10-09，asus / ROS 2 Jazzy 软件验收。真实节点与算法运行，传感器输入为明确的合成数据；尚未完成实车动态回放。

## 状态与运动联锁

`chassis_state_bridge` 将 `/state/chassis` 的 odom→base_footprint 状态转换为
Nav2 `/odom` 的 odom→base_link 状态。标定必须显式指定，两参考点之间必须为静态 TF。
桥不发布 TF；odom→base_footprint 仍只有 robot_localization 一个发布者。

完整转换包含旋转、平移杠杆臂、角速度导致的参考点线速度和两个 6×6 协方差。
单元测试通过已知偏心/旋转速度案例及六维有限差分 Jacobian 验证。
输入 frame、时间单调性、源年龄/接收年龄、有限值、四元数及正定协方差均检查。
LIO 与绝对编码器健康必须同时新鲜；任何一路失效停止输出新 `/odom`，
并使 `/state/chassis_healthy` false。EKF 持续预测不证明传感器健康。

默认输出门和任务监管都要求本地健康心跳不超过 0.25 s。
本地失效撤销许可、清空命令缓存并取消任务；恢复后不自动重放旧命令或任务。
理想底盘验收显式模拟该心跳，生产由状态桥提供，启动文件不造恒 true 发布者。
EKF 估计速度不等于 `/hardware/measured_twist`，不能用于证明实测停车。

## 原生观测与子地图

`lio_observation_adapter` 接受固定上游接口 `/lio/deskewed_odom_cloud`，输出
`/sensors/localization_observations`，消息定义在 rm_nav_interfaces。
保留源时间、mid360_main 来源、物理 LiDAR frame、标定 ID、单调序号及源参考原点。
原点按观测时刻查询 TF；resolver 与观测适配最多等待 50 ms 的 TF 到达，
仍查询原始时刻并在等待后重验新鲜度，不修改时间戳或退回最新 TF。

上游点云已经逐点去畸变并投到 odom；不能再把参考原点变换乘到点上。
上游输出没有保留逐点采集时间，因此 per_point_time 留空、reference_origin_only=true，
只声明 LOCALIZATION/RELOCALIZATION 角色。该接口不能冒充逐点射线清障输入。
原始驱动、多雷达独立观测、障碍/地形角色与逐点原点仍需补充。

`observation_submap` 检查唯一主来源、标定、角色、frame、点云存储、序号和时间。
默认滚动窗口 1 s，最多 200000 个原始点/100 帧；超限重置窗口。
确定性 0.05 m 体素后最多输出 50000 点，20 点以下不输出。
200 ms 发布周期仅发布新观测构成的子地图，保留最后源观测时间；不重发缓存并刷新时间。
输出 `/localization/odom_submap` 接入现有 GICP/KISS 链。
输入断流、状态或来源健康失效清空窗口并撤销子地图健康。
单独重启观测适配器会重置序号，需要同时重启消费窗口，不能沿用旧序号状态。

## 可复现验收

```bash
ROS_DOMAIN_ID=99 python3 tools/smoke_local_state.py
ROS_DOMAIN_ID=100 python3 tools/smoke_small_point_lio.py --with-state
ROS_DOMAIN_ID=101 python3 tools/smoke_observation_submap.py
ROS_DOMAIN_ID=90 python3 tools/smoke_motion_gate.py
ROS_DOMAIN_ID=97 python3 tools/smoke_task_resume.py
ROS_DOMAIN_ID=87 python3 tools/smoke_mppi_baseline.py
```

99：实际 TF/resolver/EKF/桥，非零参考点、位姿/协方差、编码器断流后桥停止输出。
100：实际固定版本 Point-LIO→resolver→EKF→桥→观测→子地图→真实 GICP/全局健康；
合成静态三平面、IMU 和零角编码器，检验 TF 所有权、角色/标定/时间、失流健康撤销。
101：保留源原点、禁止二次变换、窗口保留/到期，错 frame、过期、NaN、截断存储、
非法字段、错误标定/原点、多主来源、重复序号与本地故障拒绝及恢复。
90/97：本地健康失效/心跳丢失时输出与任务保持；87：真实 Nav2 理想全向模型回归。

实测标定、同步时钟、真实点云/编码器 MCAP、实车反馈和制动数据仍是硬件验收前提。
GTSAM 优化建图、TDT 与最终串口控制链仍未完成，不能据此宣布 P0–P10 全部完成。
