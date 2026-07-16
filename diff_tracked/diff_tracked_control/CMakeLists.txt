cmake_minimum_required(VERSION 3.0.2)
project(diff_tracked_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/straight_line_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
