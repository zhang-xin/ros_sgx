cmake_minimum_required(VERSION 2.8)
project(ros_sgx)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS ros_sgx-extras.cmake)

install(FILES cmake/FindSGX.cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
