#!/usr/bin/env bash
# Foreground component runner for the isolated 20261021 release.
set -eo pipefail
nav_component="${1:?driver|state|foxglove|capture|online|candidate|bag}"
shift
nav_root="${NAV_BASE:-/home/asus/nav_2027_vehicle_config_164}"
nav_release="${NAV_RELEASE:-$nav_root/releases/20261021_sensor_reference}"
nav_runtime="${NAV_RUNTIME:-$nav_root/runtime_20261021_sensor_reference}"
source "$nav_root/remote_env_20261020.bash"
if [[ "$nav_component" != driver ]]; then
  source "$nav_release/install/local_setup.bash"
fi
export ROS_DOMAIN_ID=97
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
case "$nav_component" in
  driver)
    # Keep DDS network writes out of the Livox acquisition/publish thread.
    # Only this isolated release is affected; sensor timestamps stay unchanged.
    export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
    exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
      -r __node:=nav20261021_livox -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
      -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=front_mid360 \
      -p user_config_path:="$nav_runtime/mid360_identity.json" ;;
  state)
    exec ros2 launch rm_nav_bringup vehicle_164_sensor_reference_state.launch.py \
      lio_params_file:="$nav_runtime/small_point_lio_live.yaml" ;;
  foxglove)
    exec ros2 launch rm_nav_bringup foxglove_visualization.launch.py \
      send_buffer_limit:=1048576 preview_publish_hz:=1.0 preview_max_voxels:=50000 preview_voxel:=0.15 ;;
  capture)
    exec ros2 launch rm_nav_bringup mapping_capture.launch.py \
      archive_root:="$nav_runtime/keyframes_fit153cm_debug_v3" calibration_id:=vehicle164_yaw_axis_fit153cm_debug_v3 \
      body_frame:=nav_reference state_health_topic:=/state/sensor_reference_healthy \
      rotation_trigger_source:=sensor min_keyframe_interval_s:=0.5 max_keyframe_interval_s:=2.0 ;;
  online)
    nav_session="${1:?Explicit new sensor-reference session directory required}"
    exec nice -n 10 ros2 launch rm_nav_bringup online_mapping.launch.py session:="$nav_session" \
      calibration_id:=vehicle164_yaw_axis_fit153cm_debug_v3 body_frame:=nav_reference \
      state_health_topic:=/state/sensor_reference_healthy ;;
  bag)
    mkdir -p "$nav_runtime/bags"
    exec timeout --signal=INT --kill-after=10s 180 ros2 bag record -s mcap \
      --max-cache-size 16777216 -o "$nav_runtime/bags/run_$(date +%Y%m%d_%H%M%S)" \
      /livox/lidar /livox/imu /lio/sensor_odometry /lio/deskewed_odom_cloud \
      /sensor_reference/odom /tf /tf_static /state/sensor_reference_healthy \
      /mapping/online_status /hardware/legacy_adam/adapter_status ;;
  physical)
    exec ros2 launch rm_nav_bringup legacy_adam_transport.launch.py transport:=physical \
      enable_physical_output:=true \
      serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 ;;
  navigation)
    nav_manifest="${1:?Published MapBundle manifest path required}"
    nav_profile="${2:?Explicit local_site or official required}"
    exec ros2 launch rm_nav_bringup sensor_reference_navigation.launch.py \
      map_bundle:="$nav_manifest" map_profile:="$nav_profile" ;;
  candidate)
    exec ros2 launch rm_nav_bringup legacy_adam_transport.launch.py transport:=disabled ;;
  *) echo "Unsupported component: $nav_component" >&2; exit 2 ;;
esac
