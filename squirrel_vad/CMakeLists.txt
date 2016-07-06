cmake_minimum_required(VERSION 2.8.3)
project(squirrel_vad)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  libvad
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/squirrel_vad
  CATKIN_DEPENDS roscpp rospy std_msgs libvad
  DEPENDS boost_program_options pulse pulse-simple 
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



include_directories(include ${catkin_INCLUDE_DIRS})

#paths for gmm_vad headers
include_directories(include ${PROJECT_SOURCE_DIR}/include/squirrel_vad)

#paths for pulseaudio and boost
include_directories(include /usr/local/include)
INCLUDE_DIRECTORIES(include /usr/include)

#paths for pulseaudio(only if installed) 
link_directories(/usr/local/lib)

find_library(Boost_LIBRARIES 
		NAMES boost_program_options 
		PATHS /usr/local/lib /usr/lib)
find_library(Pulse_LIBRARIES 
		NAMES pulse  
		PATHS /usr/local/lib /usr/lib)
find_library(Pulse-Simple_LIBRARIES 
		NAMES pulse-simple 
		PATHS /usr/local/lib /usr/lib)


message(STATUS ${Boost_LIBRARIES})
message(STATUS ${Pulse_LIBRARIES})
message(STATUS ${Pulse-Simple_LIBRARIES})

SET(Vad_LIBRARIES vad)

#main source file
add_executable(voice_detector ./src/voice_detector.cpp)
target_link_libraries(voice_detector ${catkin_LIBRARIES} ${Vad_LIBRARIES} ${Boost_LIBRARIES} ${Pulse_LIBRARIES} ${Pulse-Simple_LIBRARIES})

