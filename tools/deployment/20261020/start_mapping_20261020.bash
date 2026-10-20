#!/usr/bin/env bash
set -eo pipefail
base=/home/asus/nav_2027_vehicle_config_164
source "$base/remote_env_20261020.bash"
exec ros2 launch rm_nav_bringup mapping_capture.launch.py \
  archive_root:="$base/runtime_20261020/keyframes" \
  calibration_id:=vehicle164-user-mount-mid360-factory-20261013
