cmake_minimum_required(VERSION 3.0.2)
project(diff_car_base)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  urdf
  xacro
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)