cmake_minimum_required(VERSION 2.8.3)
project(ros_serial)
set(CMAKE_CXX_STANDARD 17)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)
find_package(yaml-cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_serial
#  CATKIN_DEPENDS std_msgs
#  DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/main.cpp src/ConfigParser.h src/ConfigParser.cpp src/MotorTopic.h src/MotorTopic.cpp src/Topic.h src/SerialInterface.h src/SerialInterface.cpp src/protcol_constants.h)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   yaml-cpp
)
