cmake_minimum_required(VERSION 2.8.3)
project(base_sim_controller)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        nav_msgs
        pluginlib
        pr2_controller_interface
        pr2_mechanism_model
        tf)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        base_sim_controller
    CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        pr2_controller_interface
        pr2_mechanism_model
        tf)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(base_sim_controller src/base_sim_controller.cpp)
target_link_libraries(base_sim_controller ${catkin_LIBRARIES})

install(
    TARGETS base_sim_controller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(
    DIRECTORY include/base_sim_controller/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    FILES controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
