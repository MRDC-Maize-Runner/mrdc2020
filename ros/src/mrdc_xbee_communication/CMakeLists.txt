#initialize catkin
cmake_minimum_required(VERSION 3.6)
project(mrdc_xbee_communication)
include(FindProtobuf)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial)

#find protocoll buffer
find_package(Protobuf REQUIRED)


## Declare ROS messages and services
add_message_files(FILES Analog_Controller.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include
)

#Generate protocol buffer
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
set(PROTO_SRC ${PROTO_SRC} PARENT_SCOPE)
set(PROTO_HEADER ${PROTO_HEADER} PARENT_SCOPE)
PROTOBUF_GENERATE_CPP(PROTO_SRCS1 PROTO_HDRS1 proto/state.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS2 PROTO_HDRS2 proto/status.proto)

## the nodes
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  include
)

add_executable(xbee_communication src/xbee_communication.cpp ${PROTO_SRCS1} ${PROTO_HDRS1} ${PROTO_SRCS2} ${PROTO_HDRS2})
target_link_libraries(xbee_communication ${catkin_LIBRARIES})
#add the protocol buffer
target_link_libraries(xbee_communication protobuf)

add_dependencies(xbee_communication mrdc_xbee_communication_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})