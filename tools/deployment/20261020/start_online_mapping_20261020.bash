#!/usr/bin/env bash
# Attach only: the existing recorder, LIO and Foxglove remain running.
set -eo pipefail
base=/home/asus/nav_2027_vehicle_config_164
if [[ $# != 1 || "$1" != /* || ! -d "$1" ]]; then
  echo 'Usage: bash start_online_mapping_20261020.bash /absolute/path/to/current/session' >&2
  exit 2
fi
source "$base/remote_env_20261020.bash"
source "$base/releases/20261020/install_online/local_setup.bash"
exec nice -n 10 ros2 launch rm_nav_bringup online_mapping.launch.py \
  session:="$1" calibration_id:=vehicle164-user-mount-mid360-factory-20261013
