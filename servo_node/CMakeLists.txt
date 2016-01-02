cmake_minimum_required(VERSION 2.8.3)
project(servo_node)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp segbot_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(
    servo_node
    src/Dynamixel.cpp
    src/ErrorMessage.cpp
    src/SerialDevice.cpp
    src/servo_node.cpp)
target_link_libraries(servo_node ${catkin_LIBRARIES})
add_dependencies(servo_node ${catkin_EXPORTED_TARGETS})

install(
    TARGETS servo_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
