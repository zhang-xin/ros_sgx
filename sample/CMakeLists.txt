cmake_minimum_required(VERSION 2.8)
project(ros_sgx_sample)

find_package(catkin REQUIRED COMPONENTS roscpp ros_sgx)

catkin_package(CATKIN_DEPENDS roscpp)

find_package(SGX)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${SGX_INCLUDE_DIR} ../trusted_lib ../untrusted_lib)

add_trusted_library(trusted_lib
                    SRCS ../trusted_lib/trusted_lib.cpp
                    EDL ../trusted_lib/trusted_lib.edl
                    EDL_SEARCH_PATHS ../trusted_lib)

add_enclave_library(enclave_pub
                    SRCS pub/enclave_pub.cpp
                    TRUSTED_LIBS trusted_lib
                    EDL pub/enclave_pub.edl
                    EDL_SEARCH_PATHS pub ../trusted_lib
                    LDSCRIPT pub/enclave_pub.lds)

add_enclave_library(enclave_sub
                    SRCS sub/enclave_sub.cpp
                    TRUSTED_LIBS trusted_lib
                    EDL sub/enclave_sub.edl
                    EDL_SEARCH_PATHS sub ../trusted_lib
                    LDSCRIPT sub/enclave_sub.lds)


enclave_sign(enclave_pub
             KEY pub/enclave_pub.pem
             CONFIG pub/enclave_pub.config.xml)

enclave_sign(enclave_sub
             KEY sub/enclave_sub.pem
             CONFIG sub/enclave_sub.config.xml)

add_library(untrusted_lib STATIC ../untrusted_lib/untrusted_lib.cpp)

add_untrusted_executable(pub_example
                         SRCS pub/publisher.cpp
                         EDL pub/enclave_pub.edl
                         EDL_SEARCH_PATHS pub)
add_untrusted_executable(sub_example
                         SRCS sub/subscriber.cpp
                         EDL sub/enclave_sub.edl
                         EDL_SEARCH_PATHS sub)

target_link_libraries(pub_example untrusted_lib ${catkin_LIBRARIES})
target_link_libraries(sub_example untrusted_lib ${catkin_LIBRARIES})

install(TARGETS pub_example
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS sub_example
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
