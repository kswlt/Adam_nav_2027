#!/usr/bin/env bash
set -eo pipefail
base=/home/asus/nav_2027_vehicle_config_164
runtime=$base/runtime_20261020
mkdir -p "$runtime"
python3 - <<'PY'
import json
from pathlib import Path
base=Path('/home/asus/nav_2027_vehicle_config_164')
data=json.loads((base/'config/hardware/mid360_car_164.json').read_text())
for device in data['lidar_configs']:
    device['ip']='192.168.1.152'
    device['extrinsic_parameter']={'roll':0.0,'pitch':0.0,'yaw':0.0,'x':0,'y':0,'z':0}
(base/'runtime_20261020/mid360_identity.json').write_text(json.dumps(data,indent=2)+'\n')
PY
source /opt/ros/jazzy/setup.bash
source "$base/workspace/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash"
export ROS_DOMAIN_ID=97
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=nav20261020_livox -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=front_mid360 \
  -p user_config_path:="$runtime/mid360_identity.json"
