cmake_minimum_required(VERSION 2.4)

PROJECT(GMMVAD)

# paths to include headers
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(/usr/include)

# source paths
FILE(GLOB_RECURSE SOURCES "src/webrtc/*.c*")
FILE(GLOB_RECURSE EXAMPLE "src/example/*.c*")

# library paths
LINK_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/usr/lib)

SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

SET(CMAKE_CXX_FLAGS "-o3 -w -g -std=c++11")
SET(CMAKE_CXX_LINK_FLAGS "-pg")

#We assume that boost and pulse-audio are installed
SET(Boost_LIBRARIES boost_program_options)

SET(Pulse_LIBRARIES pulse pulse-simple)

ADD_LIBRARY(gmm_vad SHARED ${SOURCES})
#ADD_LIBRARY(gmm_vad STATIC ${SOURCES})

ADD_EXECUTABLE(example ${EXAMPLE})
TARGET_LINK_LIBRARIES(example gmm_vad ${Boost_LIBRARIES} ${Pulse_LIBRARIES})

SET(CMAKE_BUILD_TYPE Release)

#install a library and header file
INSTALL(TARGETS gmm_vad DESTINATION /usr/local/lib)
