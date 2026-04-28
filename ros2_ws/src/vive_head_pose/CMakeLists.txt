cmake_minimum_required(VERSION 3.5)
project(vive_head_pose)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/vive_head_pose_node
  scripts/vive_headset_info
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
