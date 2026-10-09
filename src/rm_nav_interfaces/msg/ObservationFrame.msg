# Geometry is in header.frame_id. sensor_pose is the source origin at header.stamp.
uint8 LOCALIZATION=1
uint8 OBSTACLE=2
uint8 TERRAIN=4
uint8 CORRIDOR=8
uint8 RELOCALIZATION=16
uint8 VISUALIZATION=32
std_msgs/Header header
string source_id
string sensor_frame
string calibration_id
uint64 sequence
uint8 roles
geometry_msgs/Pose sensor_pose
sensor_msgs/PointCloud2 point_cloud
bool deskewed
bool healthy
# Empty if upstream no longer exposes each acquisition time. Never synthesize it.
float64[] per_point_time
# Such an observation cannot claim exact per-point raycast origins.
bool reference_origin_only
