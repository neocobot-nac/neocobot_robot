cmake_minimum_required(VERSION 2.8.3)
project(neo_bringup)

find_package(catkin REQUIRED COMPONENTS
  neo_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs neo_msgs tf
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include
 ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/real_rviz)