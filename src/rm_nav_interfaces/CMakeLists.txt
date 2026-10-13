cmake_minimum_required(VERSION 3.8)
project(rm_nav_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RegistrationEstimate.msg"
  "msg/ObservationFrame.msg"
  "msg/ObservationBatch.msg"
  "msg/RecoveryRequest.msg"
  "msg/RecoveryState.msg"
  "msg/TrajectoryPoint.msg"
  "msg/TimedTrajectory.msg"
  "msg/TrajectoryEnvironment.msg"
  "srv/RequestRecovery.srv"
  "srv/CommitRecovery.srv"
  "srv/ResumeRecovery.srv"
  "srv/SubmitGoal.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
