cmake_minimum_required(VERSION 2.8.3)
project(squirrel_expression)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  squirrel_hri_msgs
  sound_play
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  geometry_msgs
  roscpp 
  roslib
  sensor_msgs
  std_msgs
  tf
  squirrel_hri_msgs
  sound_play
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Expression
set(SES_SOURCES
  src/ExpressionServer.cpp)

add_executable(squirrel_expression ${SES_SOURCES})
target_link_libraries(squirrel_expression ${catkin_LIBRARIES})
add_dependencies(squirrel_expression squirrel_hri_msgs_generate_messages_cpp)

set(MSES_SOURCES 
  src/MotionExpressionServer.cpp)

add_executable(squirrel_motion_expression ${MSES_SOURCES})
target_link_libraries(squirrel_motion_expression ${catkin_LIBRARIES})
add_dependencies(squirrel_motion_expression squirrel_hri_msgs_generate_messages_cpp)

