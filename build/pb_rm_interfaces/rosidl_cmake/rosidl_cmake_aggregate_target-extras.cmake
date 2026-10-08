# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target pb_rm_interfaces::pb_rm_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${pb_rm_interfaces_TARGETS}.
if(pb_rm_interfaces_TARGETS AND NOT TARGET pb_rm_interfaces::pb_rm_interfaces)
  add_library(pb_rm_interfaces::pb_rm_interfaces INTERFACE IMPORTED)
  set_target_properties(pb_rm_interfaces::pb_rm_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${pb_rm_interfaces_TARGETS}")
endif()
