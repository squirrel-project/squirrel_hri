cmake_minimum_required(VERSION 2.8.3)
project(squirrel_expression)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
	roscpp 
	roslib
	tf
	std_msgs
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

