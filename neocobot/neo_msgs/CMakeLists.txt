cmake_minimum_required(VERSION 2.8.3)
project(neo_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
   FILES
   EndPose.msg
   JointAngles.msg
   JointVelocity.msg
   JointCurrent.msg
 )

add_service_files(FILES
  Calibrate.srv
  Forward.srv
  GetInput.srv
  GetMotorIds.srv
  Hold.srv
  Inverse.srv
  MoveToAngles.srv
  MoveToPose.srv
  Recover.srv
  Release.srv
  Reset.srv
  SetEOATAction.srv
  SetOutput.srv
  Stop.srv
)

generate_messages(DEPENDENCIES
   std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)