cmake_minimum_required(VERSION 2.8.3)
project(teleop_segbot)

set(CMAKE_BUILD_TYPE Release)

find_package(
    catkin REQUIRED
    COMPONENTS
        joy
        std_msgs
        geometry_msgs
        sensor_msgs
        pr2_controllers_msgs
        roscpp
        topic_tools)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(teleop_segbot src/teleop_segbot.cpp)
target_link_libraries(teleop_segbot ${catkin_LIBRARIES})

install(
    TARGETS teleop_segbot
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
