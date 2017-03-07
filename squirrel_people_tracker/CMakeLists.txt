cmake_minimum_required(VERSION 2.8.3)
project(squirrel_people_tracker)

find_package(catkin REQUIRED
std_srvs
roscpp
sensor_msgs
move_base_msgs
pcl_conversions pcl_ros
geometry_msgs people_msgs std_msgs tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES squirrel_people_tracker
CATKIN_DEPENDS
geometry_msgs move_base_msgs people_msgs sensor_msgs std_msgs std_srvs tf#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

#add_library(squirrel_people_tracker
#   src/squirrel_people_tracker.cpp
#)

add_library(squirrel_focus_on_head
   src/focus_head.cpp
)

#add_executable(squirrel_people_tracker_node src/squirrel_people_tracker_node.cpp)
add_executable(squirrel_focus_on_head_node src/squirrel_focus_on_head_node.cpp)
add_executable(squirrel_follow_child_node src/follow_child.cpp)

#add_dependencies(squirrel_people_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_focus_on_head_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_follow_child_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_focus_on_head ${catkin_EXPORTED_TARGETS})

#target_link_libraries(squirrel_people_tracker_node
#   squirrel_people_tracker
#)

#target_link_libraries(squirrel_people_tracker
#  ${PCL_INCLUDE_DIRS}
#  ${catkin_LIBRARIES}
#)

target_link_libraries(squirrel_focus_on_head_node
   ${catkin_LIBRARIES}
   squirrel_focus_on_head
)
target_link_libraries(squirrel_follow_child_node
   ${catkin_LIBRARIES}
)

