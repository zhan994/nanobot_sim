cmake_minimum_required(VERSION 3.0.2)
project(diff_tracked_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#c++设置项
add_executable(line_rpp_controller_node
  src/line_rpp_controller.cpp
)

add_executable(path_rpp_controller_node
  src/path_rpp_controller.cpp
)

add_dependencies(line_rpp_controller_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(path_rpp_controller_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(line_rpp_controller_node
  ${catkin_LIBRARIES}
)

target_link_libraries(path_rpp_controller_node
  ${catkin_LIBRARIES}
)

install(TARGETS
  line_rpp_controller_node
  path_rpp_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
