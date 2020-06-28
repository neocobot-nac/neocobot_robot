cmake_minimum_required(VERSION 2.8.3)
project(neo_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  sensor_msgs
  control_msgs
  industrial_msgs
  industrial_utils
  trajectory_msgs
  neo_msgs
  actionlib_msgs
  actionlib
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  sensor_msgs
  control_msgs
  industrial_msgs
  industrial_utils
  trajectory_msgs
  neo_msgs
  actionlib_msgs
  actionlib
  urdf
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
)

include_directories(include ${catkin_INCLUDE_DIRS})

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	link_directories(${catkin_LIB_DIRS} lib/lib64)
else()
	link_directories(${catkin_LIB_DIRS} lib/lib32)
endif()

## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES 
        src/NeoMainNode.cpp 
        src/NeoDriver.cpp 
        src/interfaces.cpp)
add_executable(neo_driver ${${PROJECT_NAME}_SOURCES})
add_dependencies(neo_driver neo_msgs_gencpp)
target_link_libraries(neo_driver ${catkin_LIBRARIES} pthread dl)

add_executable(follow_joint_trajectory_application applications/follow_joint_trajectory_application.cpp)
add_dependencies(follow_joint_trajectory_application neo_msgs_gencpp)
target_link_libraries(follow_joint_trajectory_application ${catkin_LIBRARIES} pthread dl)






