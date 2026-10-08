# Estimated transform maps source_frame points into header.frame_id.
# header.stamp is the source observation time, not registration completion time.
uint8 LOCAL_GICP=0
uint8 GLOBAL_KISS=1
uint8 KISS_GICP=2
uint8 AMCL_GICP=3
uint8 MANUAL_GICP=4
uint8 LOOP_KISS=5
uint8 LOOP_GICP=6
std_msgs/Header header
string source_frame
string map_version
geometry_msgs/Transform target_to_source
uint8 method
bool converged
float64 fitness
float64 residual
uint32 inlier_count
float64 inlier_ratio
float64 condition_score
float64 translation_delta
float64 rotation_delta
float64 runtime_ms
float64 confidence
float64[36] hessian
uint32 source_point_count
uint32 target_point_count
uint32 iterations
