source /opt/ros/jazzy/setup.bash
base=/home/asus/nav_2027_vehicle_config_164
source "$base/workspace/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash"
source "$base/deps/install_lio/local_setup.bash"
source "$base/releases/20261020/install/local_setup.bash"
export CMAKE_PREFIX_PATH="$base/deps_20261020/install:$base/deps/install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$base/deps_20261020/install/lib:$base/deps/install/lib:${LD_LIBRARY_PATH:-}"
if [[ -f "$base/deps_20261020/pal_install/local_setup.bash" ]]; then
  source "$base/deps_20261020/pal_install/local_setup.bash"
fi
export ROS_DOMAIN_ID=97
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
