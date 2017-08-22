cmake_minimum_required(VERSION 2.8)
project(ros_sgx_sample)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pub_example pub/publisher.cpp)

target_link_libraries(pub_example ${catkin_LIBRARIES})

install(TARGETS pub_example
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(sub_example sub/subscriber.cpp)

target_link_libraries(sub_example ${catkin_LIBRARIES})

install(TARGETS sub_example
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
