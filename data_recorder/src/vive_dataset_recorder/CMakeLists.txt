cmake_minimum_required(VERSION 3.8)
project(vive_dataset_recorder)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeadmanFrameState.msg"
  "msg/RecordingEvent.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(recorder_state_machine src/recorder_state_machine.cpp)
target_compile_features(recorder_state_machine PUBLIC cxx_std_17)
target_include_directories(recorder_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(vive_dataset_recorder_node
  src/main.cpp
  src/recorder_controller.cpp
)
set_target_properties(vive_dataset_recorder_node PROPERTIES
  OUTPUT_NAME vive_dataset_recorder
)
target_compile_features(vive_dataset_recorder_node PUBLIC cxx_std_17)
target_include_directories(vive_dataset_recorder_node PRIVATE include)
target_link_libraries(vive_dataset_recorder_node recorder_state_machine)
ament_target_dependencies(vive_dataset_recorder_node
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  sensor_msgs
  std_msgs
  tf2_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vive_dataset_recorder_node "${cpp_typesupport_target}")

install(TARGETS recorder_state_machine vive_dataset_recorder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_recorder_state_machine
    test/test_recorder_state_machine.cpp)
  target_link_libraries(test_recorder_state_machine recorder_state_machine)
endif()

ament_export_include_directories(include)
ament_package()
