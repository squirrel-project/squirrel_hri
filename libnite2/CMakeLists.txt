cmake_minimum_required(VERSION 2.8.3)
project(libnite2)

set(catkin_RUN_PACKAGES
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
	#cmake_modules
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

add_custom_target(
    build_libnite2 ALL
    COMMAND cmake -E chdir ${PROJECT_BINARY_DIR} make LIB_DIR=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION} INCLUDE_DIR=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION} PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR} -f ${PROJECT_SOURCE_DIR}/Makefile.tarball
)


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
	#INCLUDE_DIRS
	#	include
	LIBRARIES
		NiTE2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	common/include
)

add_library(NiTE2 SHARED)
set_target_properties(NiTE2 PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(build_libnite2 PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS NiTE2
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY common/lib
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
PATTERN ".svn|.so" EXCLUDE
)
