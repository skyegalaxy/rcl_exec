cmake_minimum_required(VERSION 3.20)

project(rcl_exec)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rcl REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} INTERFACE
  rcl::rcl
  # Exports the ROS C++ standard as the minimum for consumers.
  ament_cmake_ros_core::ament_ros_defaults)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(ament_cmake_ros_core rcl)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_types test/test_types.cpp)
  target_link_libraries(test_types ${PROJECT_NAME})
endif()

ament_package()
