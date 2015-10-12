cmake_minimum_required(VERSION 2.8.3)
project(servo_controller)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        pluginlib 
        pr2_controller_interface 
        pr2_mechanism_model 
        segbot_msgs 
        tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(servo_controller src/servo_controller.cpp)
target_link_libraries(servo_controller ${catkin_LIBRARIES})
