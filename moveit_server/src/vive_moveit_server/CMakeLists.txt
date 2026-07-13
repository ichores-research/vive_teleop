cmake_minimum_required(VERSION 3.8)
project(vive_moveit_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(vive_teleop_control_math src/control_math.cpp)
target_include_directories(vive_teleop_control_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(vive_teleop_control_math PUBLIC Eigen3::Eigen)

add_executable(vive_moveit_server src/vive_moveit_server.cpp)
target_include_directories(vive_moveit_server PRIVATE include)
target_link_libraries(vive_moveit_server vive_teleop_control_math Eigen3::Eigen)
ament_target_dependencies(vive_moveit_server
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  trajectory_msgs)

install(
  TARGETS vive_teleop_control_math vive_moveit_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_math test/test_control_math.cpp)
  target_link_libraries(test_control_math vive_teleop_control_math Eigen3::Eigen)
  target_include_directories(test_control_math PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(vive_teleop_control_math)
ament_package()
