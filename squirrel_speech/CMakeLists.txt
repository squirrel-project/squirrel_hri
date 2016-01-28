cmake_minimum_required(VERSION 2.8.3)
project(squirrel_speech_rec)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  squirrel_speech_msgs
  message_generation
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   squirrel_speech_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES squirrel_speech_rec
  CATKIN_DEPENDS 
    rospy 
    std_msgs
    squirrel_speech_msgs
    message_runtime
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

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(squirrel_speech_rec ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(squirrel_speech_rec_node src/squirrel_speech_rec_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(squirrel_speech_rec_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(squirrel_speech_rec_node
#   ${catkin_LIBRARIES}
# )


