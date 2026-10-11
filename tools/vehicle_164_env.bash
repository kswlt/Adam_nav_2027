# Source this file in bash; all overlays belong to the isolated vehicle workspace.
source /opt/ros/jazzy/setup.bash
export RM_NAV_DEPS_ROOT=/home/asus/nav_2027_vehicle_config_164/deps
export CMAKE_PREFIX_PATH="${RM_NAV_DEPS_ROOT}/install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${RM_NAV_DEPS_ROOT}/install/lib:${LD_LIBRARY_PATH:-}"
if [[ -f "${RM_NAV_DEPS_ROOT}/install_lio/local_setup.bash" ]]; then
  source "${RM_NAV_DEPS_ROOT}/install_lio/local_setup.bash"
fi
if [[ -f /home/asus/nav_2027_vehicle_config_164/workspace/install/local_setup.bash ]]; then
  source /home/asus/nav_2027_vehicle_config_164/workspace/install/local_setup.bash
fi
export ROS_DOMAIN_ID=97
