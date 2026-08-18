cmake_minimum_required(VERSION 3.0.2)
project(quad_uav_gazebo)

find_package(catkin REQUIRED)

catkin_package()

# Install ROS Python nodes and let catkin rewrite their shebang when needed.
catkin_install_python(PROGRAMS
  scripts/pointcloud_to_body.py
  scripts/rc_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Keep Gazebo resources available from the package share directory in an
# install space as well as directly from the source space.
install(DIRECTORY
  models
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
