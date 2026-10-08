cmake_minimum_required(VERSION 3.8)
project(rm_nav_localization)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(chassis_resolver src/chassis_resolver.cpp)
target_sources(chassis_resolver PRIVATE src/candidate_region.cpp)
target_compile_features(chassis_resolver PUBLIC cxx_std_17)
target_include_directories(chassis_resolver PUBLIC include)
target_link_libraries(chassis_resolver Eigen3::Eigen)

add_executable(chassis_resolver_test test/chassis_resolver_test.cpp)
target_link_libraries(chassis_resolver_test chassis_resolver)
add_executable(candidate_region_test test/candidate_region_test.cpp)
target_link_libraries(candidate_region_test chassis_resolver)
enable_testing()
add_test(NAME chassis_resolver_test COMMAND chassis_resolver_test)
add_test(NAME candidate_region_test COMMAND candidate_region_test)

install(TARGETS chassis_resolver chassis_resolver_test candidate_region_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
