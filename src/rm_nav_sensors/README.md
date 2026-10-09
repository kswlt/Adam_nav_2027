# rm_nav_sensors

运行节点 lio_observation_adapter 将上游已去畸变的 odom 点云转换为原生 ROS
ObservationBatch，保留源时间、标定 ID、物理 LiDAR frame 和精确时刻的源参考原点。
仅声明定位/重定位角色，不补造逐点时间，不用于逐点射线清障。
共用 point_cloud.hpp 校验字段、存储、有限坐标和点数上限。

原有 C++ ObservationFrame/ObservationBatch 是契约原型；生产使用
rm_nav_interfaces 的 ROS 消息。多雷达驱动与逐点原点仍待实现。
见 [验收记录](../../docs/state_observation_acceptance.md)。
