cmake_minimum_required(VERSION 3.8)
project(rm_nav_registration)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(registration_types INTERFACE)
target_include_directories(registration_types INTERFACE include)
target_link_libraries(registration_types INTERFACE Eigen3::Eigen)

add_executable(registration_types_test test/registration_types_test.cpp)
target_link_libraries(registration_types_test registration_types)
add_executable(localization_validator_test test/localization_validator_test.cpp)
target_link_libraries(localization_validator_test registration_types)
enable_testing()
add_test(NAME registration_types_test COMMAND registration_types_test)
add_test(NAME localization_validator_test COMMAND localization_validator_test)

install(TARGETS registration_types_test localization_validator_test
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
