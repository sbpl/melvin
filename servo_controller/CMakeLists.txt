cmake_minimum_required(VERSION 2.8.3)
project(servo_controller)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        controller_interface
        hardware_interface
        pluginlib
        segbot_msgs
        tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(servo_controller src/servo_controller.cpp)
target_link_libraries(servo_controller ${catkin_LIBRARIES})
add_dependencies(servo_controller ${catkin_EXPORTED_TARGETS})

install(
    TARGETS servo_controller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(
    DIRECTORY include/servo_controller/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    FILES controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
