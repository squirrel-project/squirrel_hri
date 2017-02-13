cmake_minimum_required(VERSION 2.8.3)
project(squirrel_ser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  squirrel_vad_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES squirrel_ser
  CATKIN_DEPENDS rospy std_msgs squirrel_vad_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

