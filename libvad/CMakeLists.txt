cmake_minimum_required(VERSION 2.8.3)
project(libvad)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src
  LIBRARIES vad
)

# paths to include headers
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_compile_options( -std=c++11 )

# source paths
FILE(GLOB_RECURSE SOURCES "${PROJECT_SOURCE_DIR}/src/webrtc/*.c*")

## Declare a C++ library
add_library(vad ${SOURCES})


###########
## Build ##
###########

## Mark executables and/or libraries for installation
install(TARGETS vad
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

