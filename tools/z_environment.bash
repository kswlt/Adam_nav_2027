#!/usr/bin/env bash
# Source, do not execute. Independent new migration prefix; no service/process changes.
nav_z_ws="${NAV_Z_WORKSPACE:-$HOME/nav_2027_vehicle_config_164/z_migration_20261023}"
nav_z_base="${NAV_BASE:-$HOME/nav_2027_vehicle_config_164}"
if [[ ! -f "$nav_z_ws/install/local_setup.bash" ]]; then
  echo "Z migration is not built: $nav_z_ws" >&2; return 2
fi
source /opt/ros/jazzy/setup.bash
if [[ -n "${NAV_LIVOX_INSTALL:-}" ]]; then
  source "$NAV_LIVOX_INSTALL/local_setup.bash"
elif [[ -f "$nav_z_base/workspace/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash" ]]; then
  source "$nav_z_base/workspace/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash"
fi
if [[ -f "$nav_z_base/deps_20261020/pal_install/local_setup.bash" ]]; then
  source "$nav_z_base/deps_20261020/pal_install/local_setup.bash"
fi
source "$nav_z_ws/install/local_setup.bash"
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib:$nav_z_base/deps_20261020/install/lib:$nav_z_base/deps/install/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${NAV_Z_DOMAIN_ID:-98}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
nav_z_actual="$(ros2 pkg prefix rm_nav_bringup)"
[[ "$nav_z_actual" == "$nav_z_ws/install/rm_nav_bringup" ]] || {
  echo "Wrong RM overlay selected: $nav_z_actual" >&2; return 2;
}
