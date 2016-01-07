cmake_minimum_required(VERSION 2.8.3)
project(segbot_gazebo)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(twist_ned_transformer src/twist_ned_transformer.cpp)
target_link_libraries(twist_ned_transformer ${catkin_LIBRARIES})

install(
    TARGETS twist_ned_transformer
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
