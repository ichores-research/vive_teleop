cmake_minimum_required(VERSION 3.0.2)
project(tiago_head_from_vive)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  trajectory_msgs
  std_msgs
  std_srvs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tiago_head_mapper
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
