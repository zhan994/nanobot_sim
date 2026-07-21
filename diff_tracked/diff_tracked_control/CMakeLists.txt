cmake_minimum_required(VERSION 3.0.2)
project(diff_tracked_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
)
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#c++设置项
add_executable(straight_line_controller_node
  src/straight_line_controller.cpp
)

add_dependencies(straight_line_controller_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(straight_line_controller_node
  ${catkin_LIBRARIES}
)

install(TARGETS straight_line_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

