cmake_minimum_required(VERSION 3.0.2)
project(diff_car_base)

find_package(catkin REQUIRED)

catkin_package()

install(
    DIRECTORY launch rviz urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
