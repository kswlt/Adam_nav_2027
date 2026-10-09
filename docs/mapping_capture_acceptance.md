# 原始关键帧采集与持久化

本步对应技术方案的 Keyframe Manager 与 original keyframe clouds 保存要求。
运行节点已实现，GTSAM、VGICP 相邻帧因子、回环和最终地图发布仍待后续接入。

## 启动与数据

先启动使用实测标定的 local_state，再单独启动建图记录进程：

```bash
ros2 launch rm_nav_bringup mapping_capture.launch.py \
  archive_root:=/absolute/path/mapping_archives \
  calibration_id:=<与local_state相同的标定ID> \
  body_frame:=base_footprint sensor_frame:=front_mid360
```

记录进程仅消费主 LIO 原生 ObservationBatch 和本地/观测健康。
通过 TF 查询观测原始时刻的 odom→base_footprint，最多等待 50 ms；
等待后重验健康/源年龄，不使用最新 TF 替代。它不发布 map→odom、不修改冻结地图。
没有标定 ID、绝对存储路径或合格输入时不记录。

第一帧创建关键帧，后续按底盘三维位移 0.15 m 或投影 yaw 10° 触发。
yaw 差采用角度环绕处理；pitch/roll 和云台/LiDAR 原点转角不冒充底盘 yaw。
底盘姿态接近 pitch ±90° 时投影 yaw 无稳定意义，该触发方式限于机器人正常底盘姿态。
只接收单一已配置主来源，校验 frame、标定、角色、源时间、序号、原点、逐点时间和 XYZ 存储。
输入断流/健康失效停止采集，不重放缓存。收集队列只保留最新观测，慢磁盘可丢采样；
本步没有声称保证每个时间阈值事件都完整记录。

每次运行创建独立 session，默认最多 5000 个关键帧/2 GiB 文件有效载荷，
另要求写入后至少保留 64 MiB 空闲。配额不跨旧 session 累计，使用前需管理整个磁盘留存。
参数 max_keyframes/max_archive_bytes 可在节点启动时显式修改，均有上限。
不能恢复追加旧 session；不要单独重启上游适配器后继续原 session，因为序号已重置。

目录内容：

```text
session_<time>_<nonce>/
  keyframe_0/
    observation.cdr
    metadata.json
  keyframe_1/
    observation.cdr
    metadata.json
```

observation.cdr 是当前 rm_nav_interfaces/msg/ObservationFrame 的完整 ROS CDR 序列化，
保留原始点云所有字段、字节、源原点和来源信息，不降采样或再次乘传感器外参。
metadata.json 记录 schema=1、关键帧 ID、源纳秒时间、标定、来源、参考点、
原始 odom_T_body 矩阵及 CDR 字节数。消费者须使用对应消息定义与 schema；
本步不是跨任意消息版本的通用 bag 格式，也没有替代原始传感器 MCAP。

上游点已经在 odom 坐标，因此优化后重建必须使用：

`optimized_T_body * inverse(recorded_odom_T_body) * original_odom_point`

这允许后端重新拼接原始关键帧，避免仅持久化一个不可逆的在线拼接地图。
后端应同样变换对应的源原点；缺失的逐点采集时间仍为空，不能用于逐点射线清障。

## 写入事务与故障

记录进程与 LIO 分离。文件先写入 .partial 临时目录，独占创建、完整写入、
文件与目录 fsync 后使用 Linux renameat2(RENAME_NOREPLACE) 原子提交，再 fsync session。
成功才发布 `/mapping/keyframe_archive` 路径；已有目录不能覆盖。
写入、配额、空间、提交失败锁存 fault，撤销 `/mapping/recorder_healthy`，不自动继续写入。
.partial 留作诊断，不能作为完整帧消费；启动新进程不会把它视为成功归档。
若 rename 已成功但后续 fsync 失败，可能存在未通知的完整目录，需人工检查后恢复。
本步没有模拟突然断电，持久化保证依赖本地 Linux 文件系统及存储设备的 fsync 语义。
录制健康仅说明此进程接受数据，不是运动许可或实测停车反馈。

## 软件验收

```bash
ROS_DOMAIN_ID=102 python3 tools/smoke_mapping_capture.py
ROS_DOMAIN_ID=103 python3 tools/smoke_small_point_lio.py --with-state --with-mapping
colcon test --packages-select rm_nav_mapping
```

102 使用明确合成底盘 TF 与观测，启动实际记录节点并检查真实文件：
云台转动不触发、底盘平移/yaw 触发，CDR 原样反序列化、源时间/标定保留、
重建移除旧位姿再使用优化位姿，错标定/frame/年龄/NaN/原点/序号/存储/多主来源拒绝，
失健康和断流拒绝，新 session 不覆盖旧文件，数量/字节配额故障保持，目录冲突不覆盖且保留 partial。
关键帧单元测试还覆盖 pitch 不误触发、yaw 环绕、非法参数/位姿和重复时间。
原有建图三个测试启用 -UNDEBUG，确保发布构建时断言仍执行。

103 使用固定版本的实际 Point-LIO、resolver、EKF、观测与 GICP 节点，
同时启动记录节点，检查收到的上游观测原样归档与断流保持。
输入仍是合成静态三平面、IMU 和编码器，不等于实车建图精度验收。
日志与实际归档样例保存在 asus 工程 log 目录，生成内容不提交到 Git。

后续：相邻关键帧 VGICP、固定稳定版 GTSAM 图优化、KISS 回环验证、
按原始关键帧重建/清理地图、官方坐标对齐与 MapBundle 原子发布。
