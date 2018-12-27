## Declare a cpp executable
add_executable(real_rviz real_rviz.cpp)
add_dependencies(real_rviz neo_msgs_gencpp)
target_link_libraries(real_rviz ${catkin_LIBRARIES})

install(TARGETS real_rviz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
