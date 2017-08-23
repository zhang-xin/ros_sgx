cmake_minimum_required(VERSION 2.8)
project(ros_sgx)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  DIRECTORY msg
  FILES sgxmsg.msg
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime
               CFG_EXTRAS ros_sgx-extras.cmake)

install(FILES cmake/FindSGX.cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
