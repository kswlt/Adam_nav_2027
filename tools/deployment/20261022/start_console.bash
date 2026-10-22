#!/usr/bin/env bash
set -e
export NAV_BASE=/home/asus/nav_2027_vehicle_config_164
export NAV_RELEASE="$NAV_BASE/releases/20261022_fit153_console"
export NAV_RUNTIME="$NAV_BASE/runtime_20261022_fit153_console"
exec python3 "$NAV_RUNTIME/nav_console.py" "$@"
