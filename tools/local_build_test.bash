#!/usr/bin/env bash
# Offline mode deliberately tests a subset; full mode never hides missing dependencies.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
profile="${1:-offline}"
case "$profile" in offline|full) ;; *) echo 'Usage: bash tools/local_build_test.bash offline|full' >&2; exit 2 ;; esac
build_root="${RM_NAV_LOCAL_BUILD_ROOT:-${HOME}/adam_nav_local_${profile}}"
source /opt/ros/jazzy/setup.bash
if [[ -n "${RM_NAV_DEPS_SETUP:-}" ]]; then source "$RM_NAV_DEPS_SETUP"; fi
mkdir -p "$build_root/reports"
cd "$build_root"
selection=()
if [[ "$profile" == offline ]]; then
  selection=(--packages-select pb_rm_interfaces rm_nav_control rm_nav_frames
    rm_nav_interfaces rm_nav_perception rm_nav_planning rm_nav_sim my_serial_py rm_nav_sensors)
fi
colcon build --base-paths "$project_root/src" "${selection[@]}" --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee reports/build.txt
source "$build_root/install/setup.bash"
colcon test --base-paths "$project_root/src" "${selection[@]}" --executor sequential \
  --return-code-on-test-failure --event-handlers console_cohesion+ 2>&1 | tee reports/test.txt
colcon test-result --verbose 2>&1 | tee reports/test-result.txt
cd "$project_root"
python3 tools/test_publish_map_bundle.py 2>&1 | tee "$build_root/reports/map-bundle-test.txt"
python3 tools/smoke_publish_map_bundle.py 2>&1 | tee "$build_root/reports/map-bundle-smoke.txt"
echo "PASS profile=$profile; reports=$build_root/reports"
[[ "$profile" != offline ]] || echo 'Subset only: hardware_interface, registration, mapping, localization, Nav2 and live LIO are NOT VERIFIED.'
