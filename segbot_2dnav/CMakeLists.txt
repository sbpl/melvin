cmake_minimum_required(VERSION 2.8.3)
project(segbot_2dnav)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        controller_manager
        hardware_interface
        roscpp
        segbot_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(control_loop src/main.cpp)
target_link_libraries(control_loop ${catkin_LIBRARIES})

install(
    TARGETS control_loop
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY maps DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY udev-rules DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
