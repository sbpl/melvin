cmake_minimum_required(VERSION 2.8.3)
project(servo_sim_controller)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        pluginlib
        pr2_controller_interface
        pr2_mechanism_model
        tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(servo_sim_controller src/servo_sim_controller.cpp)
target_link_libraries(servo_sim_controller ${catkin_LIBRARIES})

install(
    TARGETS servo_sim_controller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    FILES controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
