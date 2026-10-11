#!/usr/bin/env bash
# Startup only: no driver, TF substitutes, serial node or motor commands.
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/vehicle_164_env.bash"
export ROS_DOMAIN_ID=198
export ROS_LOCALHOST_ONLY=1
ros2 pkg executables small_point_lio
set +e
timeout --signal=INT --kill-after=3s 8s ros2 run small_point_lio small_point_lio_node --ros-args \
  --params-file "${RM_NAV_DEPS_ROOT}/src/small_point_lio/config/mid360.yaml" \
  -p lidar_type:=livox_pointcloud2 -p publish_tf:=false -p save_pcd:=false \
  -r /livox/lidar:=/test_only/no_lidar -r /livox/imu:=/test_only/no_imu
result=$?
set -e
if [[ "$result" != 124 ]]; then
  echo "FAIL: LIO exited before startup observation completed (status=$result)" >&2
  exit 1
fi
echo 'STARTUP_ONLY: process survived 8 seconds without sensor data; localization NOT TESTED.'
