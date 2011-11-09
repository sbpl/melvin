cmake_minimum_required(VERSION 2.6)
#include(rosbuild)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()

rosbuild_add_executable(segwayrmp segwayrmp.cpp segwayrmp.h rmp_frame.h  canio.h WinTypes.h  canio_rmpusb.h  canio_rmpusb.cc)
include_directories (.)

#find_library(ftd2xx ftd2xx PATHS ".")
#target_link_libraries (segwayrmp ${ftd2xx} )

message(STATUS "Architecture is ${CMAKE_HOST_SYSTEM_PROCESSOR}\n")
message(STATUS "Looking for ftd2xx_${CMAKE_HOST_SYSTEM_PROCESSOR}\n")

find_library(ftd2xx ftd2xx_${CMAKE_HOST_SYSTEM_PROCESSOR} PATHS ${segwayrmp_SOURCE_DIR})
target_link_libraries (segwayrmp ${ftd2xx} )


#message(STATUS "+++++++++++++++++++++++++++++++++++++++++++checking segwayrmp source dir = ${segwayrmp_SOURCE_DIR}\n")

link_directories (.) 
#target_link_libraries (segwayrmp ./libftd2xx.so) 
#set_target_properties(segwayrmp PROPERTIES LINK_FLAGS " -lftd2xx " )

#set_target_properties(segwayrmp PROPERTIES LINK_FLAGS " -L${segwayrmp_SOURCE_DIR} -lftd2xx -Wl,-rpath,${segwayrmp_SOURCE_DIR}/.." COMPILE_FLAGS " " )
#set_target_properties(segwayrmp PROPERTIES LINK_FLAGS " -L/home/jeep/playerstage/playerdrivers/pr_ppltracking/rosdrivers/segwayrmp -lftd2xx -Wl,-rpath,`pwd`/.." COMPILE_FLAGS " " )

rosbuild_add_executable(odom_test odom_test.cpp)
