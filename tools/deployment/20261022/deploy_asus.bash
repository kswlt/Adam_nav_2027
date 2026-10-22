#!/usr/bin/env bash
# New checkout only. Existing dirty release, original car code and MCAP stay intact.
set -eo pipefail
nav_base=/home/asus/nav_2027_vehicle_config_164
nav_release="$nav_base/releases/20261022_fit153_console"
nav_runtime="$nav_base/runtime_20261022_fit153_console"
nav_bundle="${1:?Git bundle required}"
[[ ! -e "$nav_release" && ! -e "$nav_runtime" ]] || { echo 'Target exists; inspect before retrying'; exit 1; }
git clone -b audit-p0-tf-costmap "$nav_bundle" "$nav_release"
git -C "$nav_release" remote set-url origin https://github.com/kswlt/Adam_nav_2027.git
mkdir "$nav_runtime"
cp "$nav_base/runtime_20261021_sensor_reference/mid360_identity.json" "$nav_runtime/"
cp "$nav_base/runtime_20261021_sensor_reference/small_point_lio_live.yaml" "$nav_runtime/"
install -m 755 "$nav_release/tools/deployment/20261021/vehicle_component.bash" "$nav_runtime/"
install -m 755 "$nav_release/tools/deployment/20261021/vehicle_session.py" "$nav_runtime/"
install -m 755 "$nav_release/tools/deployment/20261021/nav_console.py" "$nav_runtime/"
install -m 755 "$nav_release/tools/deployment/20261022/start_console.bash" "$nav_runtime/"
source "$nav_base/remote_env_20261020.bash"
cd "$nav_release"
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release > "$nav_runtime/build.log" 2>&1
source install/local_setup.bash
export ROS_DOMAIN_ID=198
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
nav_result=0
colcon test --executor sequential --return-code-on-test-failure > "$nav_runtime/test.log" 2>&1 || nav_result=$?
colcon test-result --verbose > "$nav_runtime/test_summary.log" 2>&1 || nav_result=$?
python3 tools/test_nav_console.py > "$nav_runtime/console_test.log" 2>&1 || nav_result=$?
git rev-parse HEAD > "$nav_runtime/deployed_commit.txt"
cat "$nav_runtime/test_summary.log"
exit "$nav_result"
