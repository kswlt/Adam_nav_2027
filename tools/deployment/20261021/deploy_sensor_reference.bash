#!/usr/bin/env bash
# ASUS-only isolated release build. Never modifies the old checkout/runtime.
set -eo pipefail
nav_root=/home/asus/nav_2027_vehicle_config_164
nav_release="$nav_root/releases/20261021_sensor_reference"
nav_runtime="$nav_root/runtime_20261021_sensor_reference"
nav_bundle="${1:?Explicit local Git bundle path required}"
[[ ! -e "$nav_release" ]] || { echo "Release already exists; inspect before retrying" >&2; exit 1; }
mkdir -p "$nav_runtime"
git clone --no-hardlinks "$nav_root/releases/20261020" "$nav_release"
git -C "$nav_release" fetch "$nav_bundle" audit-p0-tf-costmap
git -C "$nav_release" checkout -B audit-p0-tf-costmap FETCH_HEAD
git -C "$nav_release" remote set-url origin https://github.com/kswlt/Adam_nav_2027.git
git -C "$nav_release" rev-parse HEAD > "$nav_runtime/deployed_commit.txt"
cp "$nav_root/runtime_20261020/mid360_identity.json" "$nav_runtime/mid360_identity.json"
cp "$nav_root/runtime_20261020/small_point_lio_live.yaml" "$nav_runtime/small_point_lio_live.yaml"
source "$nav_root/remote_env_20261020.bash"
cd "$nav_release"
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release > "$nav_runtime/build.log" 2>&1
source "$nav_release/install/local_setup.bash"
export ROS_DOMAIN_ID=198
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
nav_test_exit=0
colcon test --executor sequential --return-code-on-test-failure > "$nav_runtime/test.log" 2>&1 || nav_test_exit=$?
colcon test-result --verbose > "$nav_runtime/test_result.log" 2>&1 || nav_test_exit=$?
cat "$nav_runtime/test_result.log"
exit "$nav_test_exit"
