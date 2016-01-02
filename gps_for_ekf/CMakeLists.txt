cmake_minimum_required(VERSION 2.8.3)
project(gps_for_ekf)

set(ROS_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS gps_common nav_msgs roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(gpsNode src/gps_for_ekf.cpp)
target_link_libraries(gpsNode ${catkin_LIBRARIES})

install(TARGETS gpsNode DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
