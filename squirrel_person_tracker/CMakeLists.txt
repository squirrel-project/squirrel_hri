cmake_minimum_required(VERSION 2.8.3)
project(squirrel_person_tracker)

set(catkin_RUN_PACKAGES
	libnite2
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  pcl_ros
  squirrel_person_tracker_msgs
  nodelet
  std_srvs
  cv_bridge
  image_transport
  sensor_msgs
  message_filters
  ${catkin_BUILD_PACKAGES}
)

# Find OpenNI2
find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2)
message(STATUS ${OpenNI2_INCLUDEDIR}) 
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1080
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib)
message(STATUS ${OpenNI2_LIBRARIES})


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tracker_bonn
  CATKIN_DEPENDS 
	${catkin_RUN_PACKAGES}
	roscpp 
	roslib
	tf
	pcl_ros
	squirrel_person_tracker_msgs	
	nodelet
	std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenNI2_INCLUDEDIR}
)

add_executable(squirrel_person_tracker src/squirrel_person_tracker_main.cpp src/floor_pointer.cpp src/squirrel_person_tracker.cpp)
target_link_libraries(squirrel_person_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES})
add_dependencies(squirrel_person_tracker ${catkin_EXPORTED_TARGETS})

add_library(shirt_detector src/people_recognition.cpp src/ImageUtils.cpp)
target_link_libraries(shirt_detector ${catkin_LIBRARIES} )
add_dependencies(shirt_detector ${catkin_EXPORTED_TARGETS})

add_executable(shirt_main src/shirt_main.cpp)
target_link_libraries(shirt_main ${catkin_LIBRARIES} shirt_detector)
add_dependencies(shirt_main ${catkin_EXPORTED_TARGETS})
#add_executable(people_recognition src/people_recognition.cpp src/ImageUtils.cpp)
#target_link_libraries(people_recognition ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES})

add_library(squirrel_person_tracker_nodelet src/squirrel_person_tracker_nodelet.cpp src/floor_pointer.cpp src/squirrel_person_tracker.cpp)
target_link_libraries(squirrel_person_tracker_nodelet ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} )
add_dependencies(squirrel_person_tracker_nodelet ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(test_floorpointer test/squirrel_person_tracker_floor_point_test.cpp src/floor_pointer.cpp)
target_link_libraries(test_floorpointer ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} )
add_dependencies(test_floorpointer ${catkin_EXPORTED_TARGETS})

add_custom_command(
    TARGET squirrel_person_tracker
    POST_BUILD
    COMMAND ${PROJECT_SOURCE_DIR}/create_nite2_link.bash
)

# for ros hydro
set_target_properties(squirrel_person_tracker PROPERTIES COMPILE_FLAGS -Dlinux) # for comipling with openni2 with linux 12.04
set_target_properties(squirrel_person_tracker_nodelet PROPERTIES COMPILE_FLAGS -Dlinux)


set(LINK_NAME $ENV{HOME}/.ros/NiTE2)
set(NITE_DIR ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/libnite2/lib/NiTE2/)
install(CODE "
		if(EXISTS ${LINK_NAME})
			message(\"Link ${LINK_NAME} to ${NITE_DIR} already exists. Re-linking ...\")
			execute_process(COMMAND rm ${LINK_NAME})
			execute_process(COMMAND ECHO Test = ${LINK_NAME})
		endif()
		execute_process(COMMAND ln -s ${NITE_DIR} ${LINK_NAME})
		message(\"Created link ${LINK_NAME} to ${NITE_DIR}.\")
	"
)
