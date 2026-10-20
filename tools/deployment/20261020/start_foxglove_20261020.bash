#!/usr/bin/env bash
set -eo pipefail
base=/home/asus/nav_2027_vehicle_config_164
source "$base/remote_env_20261020.bash"
exec ros2 launch rm_nav_bringup foxglove_visualization.launch.py \
  port:=8765 send_buffer_limit:=1048576 preview_max_voxels:=50000 preview_voxel:=0.15 preview_publish_hz:=1.0
