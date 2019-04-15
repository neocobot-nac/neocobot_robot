cmake_minimum_required(VERSION 2.8.3)
project(neo_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS plugin/joint_controllers.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
