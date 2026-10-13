# Only actual hardware measurements. The legacy my_serial_py RX does not supply
# these fields: never populate them from commanded velocity or EKF estimates.
std_msgs/Header header
uint64 sequence
geometry_msgs/Twist measured_twist
float64 yaw_rad
bool drive_ready
bool stopped
