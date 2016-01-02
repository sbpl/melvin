cmake_minimum_required(VERSION 2.8.3)
project(command_velocity_filter)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        pr2_controllers_msgs
        roscpp
        sensor_msgs
        std_msgs
        topic_tools)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(command_velocity_filter src/command_velocity_filter.cpp)
target_link_libraries(command_velocity_filter ${catkin_LIBRARIES})

install(
    TARGETS command_velocity_filter
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
