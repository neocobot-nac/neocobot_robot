cmake_minimum_required(VERSION 2.8.3)
project(neo_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs neo_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs neo_msgs
)

include_directories(include
 ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
message(STATUS " CMAKE_MODULE_PATH  ${CMAKE_MODULE_PATH}")

# find API
FIND_PACKAGE(api REQUIRED)
INCLUDE_DIRECTORIES(${libneoclient_INCLUDE_DIRS})

## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES
    src/neo_driver.cpp)
add_executable(neo_driver ${${PROJECT_NAME}_SOURCES})
add_dependencies(neo_driver neo_msgs_gencpp)
## Specify libraries to link a library or executable target against
target_link_libraries(neo_driver ${catkin_LIBRARIES} ${libneoclient} pthread)

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )

add_subdirectory(demo)
