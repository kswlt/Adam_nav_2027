#!/usr/bin/env bash
# Source Jazzy first. Build the test outside all live source/install directories.
set -euo pipefail
lio_source="${1:?upstream source directory}"
lio_install="${2:?isolated small_point_lio install prefix}"
test_work="${3:?new test work directory}"
test_tools="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
test ! -e "$test_work"
mkdir -p "$test_work"
cat > "$test_work/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(lio_dense_range_regression LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
add_executable(dense_range_test "${TEST_SOURCE}")
target_compile_features(dense_range_test PRIVATE cxx_std_17)
target_include_directories(dense_range_test PRIVATE "${LIO_SOURCE}/src" "${LIO_SOURCE}/include"
  "${LIO_SOURCE}/3rdparty" "${LIO_BINARY}/include")
target_link_libraries(dense_range_test "${LIO_LIBRARY}" Eigen3::Eigen OpenMP::OpenMP_CXX)
ament_target_dependencies(dense_range_test rclcpp)
enable_testing()
add_test(NAME actual_dense_range COMMAND dense_range_test)
CMAKE
cmake -S "$test_work" -B "$test_work/build" \
  -DTEST_SOURCE="$test_tools/test_small_point_lio_dense_range.cpp" \
  -DLIO_SOURCE="$lio_source" -DLIO_BINARY="$(dirname "$(dirname "$lio_install")")/build/small_point_lio" \
  -DLIO_LIBRARY="$lio_install/lib/libsmall_point_lio.so"
cmake --build "$test_work/build" -j2
ctest --test-dir "$test_work/build" --output-on-failure
