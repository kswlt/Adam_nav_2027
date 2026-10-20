#!/usr/bin/env bash
# New sensor-reference state only. Does not stop any existing process or open serial.
set -eo pipefail
base=/home/asus/nav_2027_vehicle_config_164
source "$base/remote_env_20261020.bash"
source "$base/releases/20261020/install_online/local_setup.bash"
params="$base/runtime_20261020/small_point_lio_live.yaml"
[[ -f "$params" ]] || { echo "Missing validated small_point_lio parameters: $params" >&2; exit 1; }
if ros2 node list 2>/dev/null | grep -Eq '^/(small_point_lio|sensor_reference|chassis_ekf|gimbal_tf)$'; then
  echo 'An existing state instance is running. Stop only your own state launch before switching profiles.' >&2
  exit 1
fi
exec ros2 launch rm_nav_bringup sensor_reference_state.launch.py lio_params_file:="$params"
