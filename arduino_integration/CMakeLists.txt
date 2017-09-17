cmake_minimum_required(VERSION 2.8.3)
project(arduino_integration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(arduino_node src/arduino_node.cpp)
target_link_libraries(arduino_node ${catkin_LIBRARIES})
add_dependencies(arduino_node arduino_integration_generate_messages_cpp)

