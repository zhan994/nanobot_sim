cmake_minimum_required(VERSION 3.0.2)
project(gazebo_worlds)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
