#!/usr/bin/env bash
# Isolated pinned upstream checkout. Does not touch an existing vehicle workspace/driver.
set -euo pipefail
nav_z_commit=76e126328b670adc92981d9b4900cc059a6f2343
nav_z_workspace="${1:?Usage: bootstrap_z_localization.bash /absolute/new/dependency_workspace}"
[[ "$nav_z_workspace" == /* ]] || { echo 'Absolute workspace required' >&2; exit 2; }
nav_z_source="$nav_z_workspace/src/z-localization"
if [[ -e "$nav_z_source" ]]; then
  [[ "$(git -C "$nav_z_source" rev-parse HEAD)" == "$nav_z_commit" ]] || {
    echo 'Existing upstream checkout is a different revision; use a new directory' >&2; exit 2;
  }
else
  mkdir -p "$nav_z_workspace/src"
  git clone https://github.com/mozihe/z-localization.git "$nav_z_source"
  git -C "$nav_z_source" checkout --detach "$nav_z_commit"
fi
git -C "$nav_z_source" diff --exit-code -- . ':!src/backend/CMakeLists.txt' ':!src/backend/src/keyframe_map.cpp' ':!src/bringup/launch/bringup.launch.py'
python3 "$(dirname "${BASH_SOURCE[0]}")/patch_z_jazzy.py" "$nav_z_source"
# Reuse the installed official driver; never compile two packages with the same name.
touch "$nav_z_source/src/livox_ros_driver2/COLCON_IGNORE"
printf 'Pinned upstream: %s\nLicense: GPL-2.0-only (retained in upstream/LICENSE)\n' "$nav_z_commit"
echo 'Source ROS Jazzy + existing Livox overlay; add GTSAM4.2 prefix, then run colcon build in this workspace.'
