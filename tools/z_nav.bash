#!/usr/bin/env bash
# Foreground, portable CLI. Source the built Z and RM overlays before invoking.
set -euo pipefail
nav_z_command="${1:-help}"
shift || true
# Keep only this CLI's own frontend/backend instances unique per ROS domain.
nav_z_lock_root="${XDG_RUNTIME_DIR:-/tmp}/adam_z_${UID}_${ROS_DOMAIN_ID:-0}"
if [[ "$nav_z_command" == state || "$nav_z_command" == mapping ]]; then
  exec 8>"${nav_z_lock_root}_frontend.lock"
  flock -n 8 || { echo 'This domain already has a z_nav state/mapping process; exit it first' >&2; exit 2; }
fi
if [[ "$nav_z_command" == mapping || "$nav_z_command" == navigation ]]; then
  exec 9>"${nav_z_lock_root}_backend.lock"
  flock -n 9 || { echo 'This domain already has a z_nav mapping/navigation backend; exit it first' >&2; exit 2; }
fi
case "$nav_z_command" in
  driver)
    nav_z_driver_config="${1:?Usage: z_nav.bash driver /absolute/mid360_identity.json}"; shift
    [[ "$nav_z_driver_config" == /* && -f "$nav_z_driver_config" ]] || { echo 'Existing absolute device/host JSON required' >&2; exit 2; }
    exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
      -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
      -p output_data_type:=0 -p frame_id:=front_mid360 -p user_config_path:="$nav_z_driver_config" "$@" ;;
  state) exec ros2 launch rm_nav_bringup vehicle_164_sensor_reference_state.launch.py "$@" ;;
  mapping)
    nav_z_output="${1:?Usage: z_nav.bash mapping /absolute/new/map_directory}"; shift
    [[ "$nav_z_output" == /* && ! -e "$nav_z_output" ]] || { echo 'Choose a new absolute map output directory' >&2; exit 2; }
    exec ros2 launch rm_nav_bringup z_localization_mapping.launch.py save_directory:="$nav_z_output" "$@" ;;
  save)
    # Keep backend alive while draining the frontend and saving its committed graph snapshot.
    python3 "$(dirname "${BASH_SOURCE[0]}")/z_save_map.py" ;;
  navigation)
    nav_z_manifest="${1:?Usage: z_nav.bash navigation /absolute/map_bundle.json local_site|official}"; shift
    nav_z_profile="${1:?Explicit map profile required}"; shift
    exec ros2 launch rm_nav_bringup sensor_reference_navigation.launch.py map_bundle:="$nav_z_manifest" map_profile:="$nav_z_profile" "$@" ;;
  foxglove)
    exec ros2 launch rm_nav_bringup foxglove_visualization.launch.py enable_live_preview:=false send_buffer_limit:=1048576 "$@" ;;
  status)
    exec ros2 topic echo /backend_node/localization_state "$@" ;;
  *)
    echo 'Source Jazzy, official Livox, Z dependency overlay and RM overlay first.'
    echo 'driver CONFIG.json | mapping NEW_DIR | save | state | navigation MANIFEST PROFILE | foxglove | status'
    echo 'Each long-running command uses its own terminal. This tool never starts serial output.' ;;
esac
