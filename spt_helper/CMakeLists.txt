cmake_minimum_required(VERSION 2.8.3)
project(spt_helper)
set(CMAKE_BUILD_TYPE Debug)

# check c++11 / c++0x
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "-std=c++11")
#   message(COMPILER_SUPPORTS_CXX11)
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "-std=c++0x")
#    message(COMPILER_SUPPORTS_CXX0X)
#else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  interactive_markers
  visualization_msgs
  geometry_msgs
  squirrel_person_tracker_msgs
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
  interactive_markers
  visualization_msgs
  geometry_msgs
  squirrel_person_tracker_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(spt_markers src/spt_markers.cpp )
target_link_libraries(spt_markers ${catkin_LIBRARIES})
add_dependencies(spt_markers ${catkin_EXPORTED_TARGETS})

add_executable(spt_skeleton src/spt_skeleton_lines.cpp)
target_link_libraries(spt_skeleton ${catkin_LIBRARIES})
add_dependencies(spt_skeleton ${catkin_EXPORTED_TARGETS})

add_executable(spt_user_info src/spt_user_info.cpp)
target_link_libraries(spt_user_info ${catkin_LIBRARIES})
add_dependencies(spt_user_info ${catkin_EXPORTED_TARGETS})
