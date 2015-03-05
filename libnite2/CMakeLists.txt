cmake_minimum_required(VERSION 2.8.3)
project(libnite2)

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
	LIBRARIES
		${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libNiTE2.so
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_library(NiTE2 SHARED IMPORTED)
set_target_properties(NiTE2 PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libNiTE2.so)
set_target_properties(NiTE2 PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

install(DIRECTORY 
        ${CATKIN_DEVEL_PREFIX}/include/libnite2/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

FILE(GLOB INI_FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/*.ini)
install(FILES 
        ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libNiTE2.so
	${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libNiTE2.jni.so
	${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenNI2.so
	${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenNI2.so.0
        ${INI_FILES}
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY
       ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/NiTE2
       ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/OpenNI2
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
