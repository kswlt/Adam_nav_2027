#!/usr/bin/env bash
set -eo pipefail
export MAKEFLAGS="${MAKEFLAGS:--j2}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/vehicle_164_env.bash"
cd /home/asus/nav_2027_vehicle_config_164/workspace
bash tools/bootstrap_small_point_lio.sh
# The raw-state Point-LIO overlay is built separately from the navigation packages.
source "${RM_NAV_DEPS_ROOT}/install_lio/local_setup.bash"
mapfile -t nav_packages < <(colcon list --names-only | grep -E '^(rm_nav_|my_serial_py$|pb_rm_interfaces$)')
colcon build --packages-select "${nav_packages[@]}" --executor sequential --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS= \
  "-Dsmall_gicp_DIR=${RM_NAV_DEPS_ROOT}/install/lib/cmake/small_gicp" \
  "-Drobin_DIR=${RM_NAV_DEPS_ROOT}/install/lib/cmake/robin" \
  "-Dkiss_matcher_DIR=${RM_NAV_DEPS_ROOT}/install/lib/cmake/kiss_matcher"
colcon test --packages-select "${nav_packages[@]}" --executor sequential
colcon test-result --verbose
