# 车载隔离部署修复报告

日期：2026-10-11。检查基线：`b84f024a03691841abdd1308a19e7b1446df5767`。
已 fetch；远端 feature 仍为该提交，main 为 `7e2afa126988864ff10b5e6e8450f269eb7d3e8e`。

## 发现与修复

| 问题 | 根因与影响 | 修复 |
| --- | --- | --- |
| 车载部署说明写 Humble | 与实际 `/opt/ros/jazzy` 不符，可能加载错误环境 | profile 和说明改为 Jazzy |
| 编译原版 point_lio 被当作启动依赖就绪 | local_state 启动的是 small_point_lio | 独立构建固定版本与仓库 raw-state 补丁，提供 overlay 环境脚本 |
| 安装位置写入驱动点云变换 | 后续安装 TF 可能再次应用相同位移；IMU 与点云坐标处理不一致 | JSON 改为恒等变换，安装位置只保留在车体配置/TF；尚未完成本车 TF 联合验收 |
| 声称驱动自动给 LIO 提供工厂外参 | LIO 实际读取 extrinsic_T/extrinsic_R | 更正说明，内部外参来源及方向待核验 |
| optimizer 缺 small_gicp 头文件 | 直接使用 small_gicp，但仅通过 PRIVATE 后端间接链接 | 显式 find_package 和链接导入目标 |
| 去掉全局 -I 后仍构建失败 | 旧 CMake 编译器探测缓存将依赖目录记为 implicit include | 原缓存移动至隔离根目录 mapping_build_cache_before_fix；重新配置构建 |
| Git 作者配置回退 | 此车仓库 user.name/email 仍为 Akaneee126 | 只修改当前仓库为 kswlt / kswlt@users.noreply.github.com，不重写历史 |

## 执行与结果

平台：`192.168.1.164:2233`；工作区 `/home/asus/nav_2027_vehicle_config_164/workspace`。

```bash
bash tools/vehicle_164_build.bash
bash tools/vehicle_164_lio_startup_smoke.bash
```

构建脚本运行固定版本 small_point_lio 构建、colcon build、colcon test 和
colcon test-result --verbose。small_point_lio 单独构建通过，导航/通信 13 包构建通过；
40 tests，0 errors，0 failures，1 skipped。
首次清理旧缓存后的构建通过，随后用最终源码重跑验证。

启动 smoke：ROS_DOMAIN_ID=198、localhost discovery，8 秒无数据运行正常并受控结束。
不启动串口、驱动、车体 TF 或运动输出。此结果仅为 STARTUP_ONLY。

日志位于隔离根目录 `vehicle_build_current.log`、`lio_startup_current.log`。
复用官方 ROS 2 Jazzy/colcon、small_gicp 导入目标和现有固定版本 small_point_lio。

## 未验证与边界

- 网口实测 carrier=0：真实点云、IMU、实时 LIO/TF、Foxglove 时延均 NOT TESTED。
- 工厂内部外参、安装 TF 的生产配置、车体参考点与单 yaw 语义尚需联合验证。
- Omni 适配器超时/反馈处理和 ros2_control 实际 IO 尚未验收。
- 未连接物理输出，未启动串口，未改变原导航进程。
- 保留原有未提交 serialpy_node.py、livox_ros_driver2 和 point_lio 副本，不纳入本次提交。
- 恒等驱动变换是后续车体 TF 的输入契约；依赖驱动直接输出车体坐标的旧启动方式不得混用。
- Git 提交与远端分支信息以本报告所属提交及推送后 ls-remote 为准。
