# rm_nav_mapping

Keyframes, submaps, loop closure, pose graph and MapBundle.

This package is part of the RM Nav V2 staged implementation.

运行节点 `keyframe_recorder` 从主 LIO 观测与精确时刻底盘 TF 采集关键帧。
原始 ObservationFrame CDR 与对应 odom_T_body 持久化到独立 session，
数量/空间/写入故障锁存停止。优化后以新位姿乘旧位姿的逆重建原始点云。
独立启动 `mapping_capture.launch.py`；不修改 match 模式 TF。
见 [采集验收](../../docs/mapping_capture_acceptance.md)。

`KeyframeManager` triggers from chassis translation/yaw. LiDAR yaw is deliberately not used because the main sensor may be mounted on the moving big gimbal.

`LoopCandidateManager` first filters by time separation and rough spatial proximity. It only produces candidates; KISS, GICP and LoopValidator must approve a loop before GTSAM receives a factor.

`MapBundle` is the metadata contract for frozen localization maps, occupancy maps and terrain data. Runtime match mode must consume a versioned bundle and never silently mix calibration or map versions.
