cmake_minimum_required(VERSION 3.8)
project(vive_moveit_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(vive_moveit_server src/vive_moveit_server.cpp)
target_compile_features(vive_moveit_server PUBLIC cxx_std_17)
ament_target_dependencies(
  vive_moveit_server
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  trajectory_msgs
)

install(TARGETS vive_moveit_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
