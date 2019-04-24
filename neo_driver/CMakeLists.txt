cmake_minimum_required(VERSION 2.8.3)
project(neo_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	link_directories(${catkin_LIB_DIRS} lib/lib64)

else()
	link_directories(${catkin_LIB_DIRS} lib/lib32)
endif()

## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES
    src/NeoDriver.cpp
)
add_executable(neo_driver ${${PROJECT_NAME}_SOURCES})

## Specify libraries to link a library or executable target against
target_link_libraries(neo_driver ${catkin_LIBRARIES} pthread dl)
