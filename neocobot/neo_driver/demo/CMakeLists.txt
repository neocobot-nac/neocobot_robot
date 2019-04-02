## Declare a cpp executable
add_executable(TrajMoveDemo TrajMoveDemo.cpp)
add_dependencies(TrajMoveDemo neo_msgs_gencpp)
target_link_libraries(TrajMoveDemo ${catkin_LIBRARIES})

install(TARGETS TrajMoveDemo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
