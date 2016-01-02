cmake_minimum_required(VERSION 2.8.3)
project(exploration_planner)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(sbpl REQUIRED)

find_package(
    catkin REQUIRED
    COMPONENTS actionlib_msgs costmap_2d nav_core nav_msgs pluginlib roscpp tf)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        costmap_2d
        nav_core
    DEPENDS
        sbpl)

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(${PROJECT_NAME} src/exploration_planner.cpp src/global_planner.cpp src/astarpoint.cpp src/raycaster.cpp src/filetransfer.cpp)
target_link_libraries(${PROJECT_NAME} ${SBPL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(send_goals src/send_dummy_goals.cpp)
target_link_libraries(send_goals ${catkin_LIBRARIES})

install(
    TARGETS ${PROJECT_NAME} send_goals
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(
    DIRECTORY include/exploration_planner/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES bgp_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
