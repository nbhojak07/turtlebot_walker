cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_walker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Declare catkin package
catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(walker src/main.cpp src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})


