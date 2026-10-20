#!/usr/bin/env bash
set -eo pipefail
base=/home/asus/nav_2027_vehicle_config_164/deps_20261020
source /opt/ros/jazzy/setup.bash
mkdir -p "$base/pal_ws/src/pal_statistics_msgs/msg"
cp /opt/ros/jazzy/share/pal_statistics_msgs/msg/*.msg "$base/pal_ws/src/pal_statistics_msgs/msg/"
cp /opt/ros/jazzy/share/pal_statistics_msgs/package.xml "$base/pal_ws/src/pal_statistics_msgs/"
cat > "$base/pal_ws/src/pal_statistics_msgs/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.8)
project(pal_statistics_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Statistic.msg" "msg/Statistics.msg" "msg/StatisticsNames.msg" "msg/StatisticsValues.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
CMAKE
cd "$base/pal_ws"
export MAKEFLAGS=-j2
colcon build --executor sequential --merge-install --install-base "$base/pal_install" --cmake-args -DCMAKE_BUILD_TYPE=Release
echo PAL_MESSAGE_REBUILD_PASSED
