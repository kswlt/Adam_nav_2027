#!/usr/bin/env bash
set -e
nav_operator_ws="${NAV_Z_WORKSPACE:-$HOME/nav_2027_vehicle_config_164/z_migration_20261023}"
nav_operator_runtime="${NAV_Z_RUNTIME:-$nav_operator_ws/runtime_live_20261023}"
mkdir -p "$nav_operator_runtime"
# Lock belongs only to the console, never to the active mapping process.
exec 7>"$nav_operator_runtime/operator.lock"
flock -n 7 || { echo 'Z operator console already running'; exit 0; }
source "$nav_operator_ws/src/Adam_nav_2027/tools/z_environment.bash"
exec python3 "$nav_operator_ws/src/Adam_nav_2027/tools/z_operator/server.py" \
    --workspace "$nav_operator_ws" --runtime "$nav_operator_runtime"
