cmake_minimum_required(VERSION 3.8)
project(rm_nav_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RegistrationEstimate.msg"
  "msg/RecoveryRequest.msg"
  "msg/RecoveryState.msg"
  "srv/RequestRecovery.srv"
  "srv/CommitRecovery.srv"
  "srv/ResumeRecovery.srv"
  "srv/SubmitGoal.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
