# ROS-SGX

A ROS package ros_sgx is provided for using SGX inside ROS.

To use it, clone the repo to catkin_ws/src, and in the projects which need to use it:
```
# add in package.xml
<build_depend>ros_sgx</build_depend>

# add in CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS ros_sgx)
find_package(SGX)
```

The following variables are provided:
- SGX_HW # ON for hardware, OFF for simulation.
- SGX_MODE # Debug; PreRelease; Release.
- SGX_INCLUDE_DIR
- SGX_TLIBC_INCLUDE_DIR
- SGX_LIBCXX_INCLUDE_DIR
- SGX_INCLUDE_DIRS # all 3 dirs above
- SGX_LIBRARY_DIR

The following functions are provided:
```
# build enclave library
add_enclave_library(target
                    [USE_PREFIX]
                    SRCS src_file1 src_file2 ...
                    EDL edl_file
                    EDL_SEARCH_PATHS path1 path2 ...
                    [TRUSTED_LIBS lib1 lib2 ...]
                    [LDSCRIPT ld_script_file])

# build trusted static library to be linked into enclave library
add_trusted_library(target
                    [USE_PREFIX]
                    SRCS src_file1 src_file2 ...
                    EDL edl_file
                    EDL_SEARCH_PATHS path1 path2 ...
                    [LDSCRIPT ld_script_file])

# sign the enclave, according to configurations one-step or two-step signing will be performed.
# default one-step signing output enclave name is target.signed.so, change it with OUTPUT option.
enclave_sign(target
             KEY key
             CONFIG config
             [OUTPUT file_name])

# build untrusted executable to run with enclave
add_untrusted_executable(target
                         [USE_PREFIX]
                         SRCS src1 src2 ...
                         EDL edl_file1 edl_file2 ...
                         EDL_SEARCH_PATHS path1 path2 ...)

# build untrusted library to be run with enclave
add_untrusted_library(target
                      STATIC | SHARED | MODULE
                      [USE_PREFIX]
                      SRCS src1 src2 ...
                      EDL edl_file1 edl_file2 ...
                      EDL_SEARCH_PATHS path1 path2 ...)
```

## Trusted/untrusted library for sealing/unsealing msg

A pair of helper library is provided for encrypt messages. Developer can use provided functions to encrypt messages with SGX SDK seal operation, then the encrypted message could be passed with original roscpp, athe the subscriber the message could be decrypted in enclave. A new SGX msg format is provided in ros_sgx package. The message is generated/consumed in both sides of pub/sub so it's never plaintext outside enclaves.

The message format is controlled by the developer, what the provided library operates is just a buffer.

Currently only single machine message passing is supported because SGX seal/unseal should be using the same CPU.

## Sample

A sample is provided for demonstrating the ros_sgx package and helper library usage.
