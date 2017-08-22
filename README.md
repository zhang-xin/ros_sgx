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
