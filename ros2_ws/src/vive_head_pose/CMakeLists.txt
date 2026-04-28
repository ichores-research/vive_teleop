cmake_minimum_required(VERSION 3.5)
project(vive_head_pose)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# OpenVR: built and installed by ../docker or system (CMAKE_PREFIX_PATH)
find_path(OPENVR_INCLUDE_DIR openvr.h
  PATHS /usr/local/include/openvr /usr/include/openvr ENV OPENVR_INCLUDE_DIR
)
find_library(OPENVR_LIBRARY openvr_api
  PATHS /usr/local/lib /usr/lib ENV OPENVR_LIB_DIR
)

if(NOT OPENVR_INCLUDE_DIR OR NOT OPENVR_LIBRARY)
  message(FATAL_ERROR "OpenVR not found. Install OpenVR headers/libs (e.g. build ValveSoftware/openvr with CMAKE_INSTALL_PREFIX=/usr/local).")
endif()

include_directories(${OPENVR_INCLUDE_DIR})

add_executable(vive_head_pose_node src/vive_head_pose_node.cpp)
ament_target_dependencies(vive_head_pose_node
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(vive_head_pose_node ${OPENVR_LIBRARY})

install(TARGETS vive_head_pose_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
