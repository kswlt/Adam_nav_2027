# rm_nav_mapping

Keyframes, submaps, loop closure, pose graph and MapBundle.

This package is part of the RM Nav V2 staged implementation.

运行节点 `keyframe_recorder` 从主 LIO 观测与精确时刻底盘 TF 采集关键帧。
原始 ObservationFrame CDR 与对应 odom_T_body 持久化到独立 session，
数量/空间/写入故障锁存停止。优化后以新位姿乘旧位姿的逆重建原始点云。
独立启动 `mapping_capture.launch.py`；不修改 match 模式 TF。
见 [采集验收](../../docs/mapping_capture_acceptance.md)。

`offline_graph_optimizer <session> <新输出目录>` 读取停止录制的归档，使用真实 VGICP
生成相邻 Pose3 因子，由固定版 GTSAM 优化，并从原始关键帧重新拼接点云。
归档 SHA256 与录制锁用于一致性检查，拒绝不合格边和已有输出。
输出 mapping_odom 坐标；未接入回环、官方对齐或 MapBundle 发布。
见 [离线建图验收](../../docs/offline_mapping_acceptance.md)。

`KeyframeManager` triggers from chassis translation/yaw. LiDAR yaw is deliberately not used because the main sensor may be mounted on the moving big gimbal.

`LoopCandidateManager` first filters by time separation and rough spatial proximity. It only produces candidates; KISS, GICP and LoopValidator must approve a loop before GTSAM receives a factor.

`MapBundle` is the metadata contract for frozen localization maps, occupancy maps and terrain data. Runtime match mode must consume a versioned bundle and never silently mix calibration or map versions.
