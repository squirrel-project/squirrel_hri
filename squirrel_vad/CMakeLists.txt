cmake_minimum_required(VERSION 2.8.3)
project(squirrel_vad)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  libvad
  squirrel_vad_msgs
)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/squirrel_vad
  CATKIN_DEPENDS roscpp std_msgs libvad squirrel_vad_msgs
  DEPENDS boost_program_options pulse pulse-simple 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(include ${PROJECT_SOURCE_DIR}/include/squirrel_vad)


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

#SET(Vad_LIBRARIES vad)

#main source file
add_executable(voice_detector ./src/voice_detector.cpp)
target_link_libraries(voice_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Pulse_LIBRARIES} ${Pulse-Simple_LIBRARIES})
#target_link_libraries(voice_detector ${catkin_LIBRARIES} ${Vad_LIBRARIES} ${Boost_LIBRARIES} ${Pulse_LIBRARIES} ${Pulse-Simple_LIBRARIES})

