# rm_nav_mapping

Keyframes, submaps, loop closure, pose graph and MapBundle.

This package is part of the RM Nav V2 staged implementation.

`KeyframeManager` triggers from chassis translation/yaw. LiDAR yaw is deliberately not used because the main sensor may be mounted on the moving big gimbal.

`LoopCandidateManager` first filters by time separation and rough spatial proximity. It only produces candidates; KISS, GICP and LoopValidator must approve a loop before GTSAM receives a factor.

`MapBundle` is the metadata contract for frozen localization maps, occupancy maps and terrain data. Runtime match mode must consume a versioned bundle and never silently mix calibration or map versions.
