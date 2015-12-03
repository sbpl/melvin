cmake_minimum_required(VERSION 2.8.3)
project(segbot_ipc)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

#rosbuild_add_executable(segbot_ipc src/segbot_ipc.cpp)
#target_link_libraries(segbot_ipc /usr/local/lib/libipc.a)
