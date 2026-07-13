cmake_minimum_required(VERSION 3.0.2)
project(diff_car_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(
    DIRECTORY config launch rviz worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
