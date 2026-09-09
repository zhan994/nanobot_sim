cmake_minimum_required(VERSION 3.0.2)
project(diff_tracked_planning)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

# Use this workspace's installed copy. This also avoids accidentally selecting
# the incomplete package config generated inside f2c_build.
get_filename_component(F2C_LOCAL_PREFIX
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../f2c_install" ABSOLUTE)
if(EXISTS "${F2C_LOCAL_PREFIX}/lib/cmake/Fields2Cover/Fields2CoverConfig.cmake")
  set(Fields2Cover_DIR
    "${F2C_LOCAL_PREFIX}/lib/cmake/Fields2Cover"
    CACHE PATH "Fields2Cover package directory" FORCE)
endif()
find_package(Fields2Cover REQUIRED CONFIG)

catkin_package(
  CATKIN_DEPENDS
    nav_msgs
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(f2c_coverage_planner_node
  src/f2c_coverage_planner_node.cpp
)

add_dependencies(f2c_coverage_planner_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(f2c_coverage_planner_node
  ${catkin_LIBRARIES}
  Fields2Cover::Fields2Cover
)

# Fields2Cover's fetched dependencies are installed beside the main library.
# Keep that directory available to the runtime linker.
get_filename_component(F2C_INSTALL_PREFIX
  "${Fields2Cover_DIR}/../../.." ABSOLUTE)
set_target_properties(f2c_coverage_planner_node PROPERTIES
  BUILD_RPATH "${F2C_INSTALL_PREFIX}/lib"
  INSTALL_RPATH "${F2C_INSTALL_PREFIX}/lib"
  # DT_RPATH is also used to resolve Fields2Cover's indirect dependencies,
  # including the locally installed OR-Tools shared library.
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

install(TARGETS f2c_coverage_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
