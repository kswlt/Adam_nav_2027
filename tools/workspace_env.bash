# Source this file from Bash. It never changes the selected ROS domain or starts nodes.
rm_nav_script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RM_NAV_DEPS_ROOT="${RM_NAV_DEPS_ROOT:-${HOME}/.local/share/adam_nav/deps}"
export RM_NAV_BUILD_ROOT="${RM_NAV_BUILD_ROOT:-${rm_nav_script_root}}"
rm_nav_restore_nounset=false
[[ $- != *u* ]] || rm_nav_restore_nounset=true
set +u
source /opt/ros/jazzy/setup.bash
for rm_nav_overlay in "$RM_NAV_DEPS_ROOT/livox_ws/install" "$RM_NAV_DEPS_ROOT/install_lio" "$RM_NAV_BUILD_ROOT/install"; do
  if [[ -f "$rm_nav_overlay/local_setup.bash" ]]; then source "$rm_nav_overlay/local_setup.bash"; fi
done
if [[ -d "$RM_NAV_DEPS_ROOT/install" ]]; then
  export CMAKE_PREFIX_PATH="$RM_NAV_DEPS_ROOT/install:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$RM_NAV_DEPS_ROOT/install/lib:${LD_LIBRARY_PATH:-}"
fi
[[ $rm_nav_restore_nounset != true ]] || set -u
unset rm_nav_script_root rm_nav_restore_nounset rm_nav_overlay
