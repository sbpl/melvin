cmake_minimum_required(VERSION 2.8.3)
project(segbot_stage)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp spellbook_ros)

find_package(PkgConfig REQUIRED)
pkg_check_modules(sdl REQUIRED sdl)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${sdl_LIBRARIES})

add_executable(teleop_keyboard src/teleop_pr2_keyboard.cpp)
target_link_libraries(teleop_keyboard ${catkin_LIBRARIES})

add_executable(teleop_joystick src/teleop_joystick.cpp)
target_link_libraries(teleop_joystick ${sdl_LIBRARIES} ${catkin_LIBRARIES})
