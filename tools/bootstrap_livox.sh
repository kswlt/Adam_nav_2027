#!/usr/bin/env bash
set -euo pipefail
# Separate, pinned ROS overlay; no upstream build.sh (which deletes build/install).
root="${RM_NAV_DEPS_ROOT:-${HOME}/.local/share/adam_nav/deps}"
sdk="$root/src/Livox-SDK2"
driver="$root/livox_ws/src/livox_ros_driver2"
pin_checkout() {
  local path="$1" url="$2" revision="$3"
  if [[ ! -d "$path" ]]; then git clone "$url" "$path"; fi
  # package.xml is generated below, so only that known generated file is ignored.
  if [[ -n "$(git -C "$path" status --porcelain --untracked-files=no)" ]]; then
    echo "Refusing to overwrite upstream modifications: $path" >&2; exit 1
  fi
  if ! git -C "$path" cat-file -e "$revision^{commit}" 2>/dev/null; then
    git -C "$path" fetch origin "$revision"
  fi
  git -C "$path" checkout --detach "$revision"
}
mkdir -p "$root/src" "$root/livox_ws/src"
pin_checkout "$sdk" https://github.com/Livox-SDK/Livox-SDK2.git c0796f04c143143899c87a773d9f6b7136453c0b
pin_checkout "$driver" https://github.com/Livox-SDK/livox_ros_driver2.git 21445540f0d100dc86a7e6df312dd70bbdb4afdf
cmake -S "$sdk" -B "$root/build/livox_sdk2" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/install" \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build "$root/build/livox_sdk2" -j2
cmake --install "$root/build/livox_sdk2"
if [[ -e "$driver/package.xml" ]] && ! cmp -s "$driver/package.xml" "$driver/package_ROS2.xml"; then
  echo 'Existing package.xml differs from the ROS2 manifest; refusing overwrite.' >&2; exit 1
fi
cp "$driver/package_ROS2.xml" "$driver/package.xml"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export CMAKE_PREFIX_PATH="$root/install:${CMAKE_PREFIX_PATH:-}"
cd "$root/livox_ws"
MAKEFLAGS=-j2 colcon build --packages-select livox_ros_driver2 --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH="$root/install/lib"
