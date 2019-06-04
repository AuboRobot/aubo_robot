cmake_minimum_required(VERSION 2.8.3)
project(aubo_controller)



find_package(catkin REQUIRED COMPONENTS aubo_msgs  moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs
  control_msgs trajectory_msgs actionlib_msgs actionlib
  urdf industrial_msgs industrial_utils)

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    find_package(Eigen3 REQUIRED)
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Eigen3 Package in aubo_controller for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    find_package(Eigen REQUIRED)
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Eigen Package in aubo_controller for  for indigo")
endif()



catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    control_msgs
    actionlib
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    trajectory_msgs
    industrial_msgs
    actionlib
    actionlib_msgs
    industrial_utils
    urdf
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_dummy
)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#add_definitions(-std=c++11)

#add_library(aubo_Trajectory_Planning_Node_Class
#    include/aubo_trajectory_planning_class.h
#    src/aubo_trajectory_planning_class.cpp)



include_directories(include  ${catkin_INCLUDE_DIRS})


if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find EIGEN3_INCLUDE_DIRS Package in aubo_controller for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find EIGEN_INCLUDE_DIRS Package in aubo_controller for  for indigo")
endif()

link_directories(${catkin_LIBRARY_DIRS} )

ADD_EXECUTABLE(aubo_joint_trajectory_action src/joint_trajectory_action.cpp)
TARGET_LINK_LIBRARIES(aubo_joint_trajectory_action industrial_robot_client ${catkin_LIBRARIES})
install(TARGETS aubo_joint_trajectory_action DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(aubo_Trajectory_Planning_Node src/aubo_trajectory_planning.cpp)
#target_link_libraries(aubo_Trajectory_Planning_Node   ${catkin_LIBRARIES})

#ADD_EXECUTABLE(aubo_robot_simulator src/aubo_robot_simulator.cpp)
#TARGET_LINK_LIBRARIES(aubo_robot_simulator industrial_robot_client ${catkin_LIBRARIES})
#install(TARGETS aubo_robot_simulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS script/aubo_controller/aubo_robot_simulator.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

message(STATUS ")))))" ${CATKIN_PACKAGE_BIN_DESTINATION})
