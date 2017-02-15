cmake_minimum_required(VERSION 2.8.3)
project(squirrel_speech_rec)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  squirrel_speech_msgs
  squirrel_hri_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES squirrel_speech_rec
  CATKIN_DEPENDS 
    rospy 
    std_msgs
    squirrel_speech_msgs
    squirrel_hri_msgs
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

add_executable(speech_visu src/speech_visu.cpp)
target_link_libraries(speech_visu ${catkin_LIBRARIES})

add_dependencies(speech_visu squirrel_speech_msgs generate_messages_cpp)
