# header.stamp is the execution epoch. IDs strictly increase per producer session.
std_msgs/Header header
uint64 trajectory_id
string map_bundle_version
string environment_version
builtin_interfaces/Time valid_until
TrajectoryPoint[<=200000] points
