cmake_minimum_required(VERSION 2.8.3)
project(segbot_2dnav)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        pr2_controller_manager
        pr2_hardware_interface
        segbot_msgs
        segway_rmp
        segbot_description
        servo_controller
        servo_node)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(controller_manager src/main.cpp)
target_link_libraries(controller_manager ${catkin_LIBRARIES})

install(
    TARGETS controller_manager
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY maps DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY udev-rules DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
