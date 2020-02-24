cmake_minimum_required(VERSION 3.1)
project(mrdc_controls_processing)

#needed for std::clamp
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

#find protocoll buffer


## Declare ROS messages and services
add_message_files(FILES Analog_Controller.msg)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include
)

#Generate protocol buffer

## the nodes
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  include
)

add_executable(controls_processing src/controls_processing.cpp)
target_link_libraries(controls_processing ${catkin_LIBRARIES})
#add the protocol buffer
target_link_libraries(controls_processing protobuf)