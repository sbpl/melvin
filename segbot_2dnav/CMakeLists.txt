cmake_minimum_required(VERSION 2.8.3)
project(segbot_2dnav)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        pr2_controller_manager
        pr2_hardware_interface
        segbot_msgs
        segway_rmp
        segway_robot
        servo_controller
        servo_node)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(controller_manager src/main.cpp)
target_link_libraries(controller_manager ${catkin_LIBRARIES})
