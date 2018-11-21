cmake_minimum_required(VERSION 2.8.3)
project(aubo_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
   FILES
   Analog.msg
   Digital.msg
   IOState.msg
   JointPos.msg
   TraPoint.msg
   GoalPoint.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SetPayload.srv
   SetIO.srv
   GetFK.srv
   GetIK.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
