#!/usr/bin/env bash
set -eo pipefail
base=/home/asus/nav_2027_vehicle_config_164
source "$base/remote_env_20261020.bash"
python3 - <<'PY'
from pathlib import Path
import yaml
base=Path('/home/asus/nav_2027_vehicle_config_164')
data=yaml.safe_load((base/'deps/src/small_point_lio/config/mid360.yaml').read_text())
data['small_point_lio']['ros__parameters'].update({'lidar_type':'livox_pointcloud2','lidar_topic':'/sensors/front_mid360/guarded_points','imu_topic':'/livox/imu','save_pcd':False,'max_distance':30.,'acc_norm':1.})
(base/'runtime_20261020/small_point_lio_live.yaml').write_text(yaml.safe_dump(data))
PY
exec ros2 launch rm_nav_bringup local_state.launch.py \
  calibration_file:="$base/releases/20261020/src/rm_nav_frames/config/vehicle_164_fixed_mid360.yaml" \
  lio_params_file:="$base/runtime_20261020/small_point_lio_live.yaml" \
  allow_user_mount:=true enable_mid360_guard:=true
