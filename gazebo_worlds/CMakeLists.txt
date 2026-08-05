cmake_minimum_required(VERSION 3.0.2)
project(gazebo_worlds)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch worlds models imgs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
