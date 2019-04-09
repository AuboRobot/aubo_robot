cmake_minimum_required(VERSION 2.8.3)
project(aubo_controller)

find_package(catkin REQUIRED COMPONENTS aubo_msgs  moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs
  control_msgs trajectory_msgs actionlib_msgs actionlib
  urdf industrial_msgs industrial_utils)

#find_package(Eigen REQUIRED)

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

#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "-std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "-std=c++0x")
#else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
#endif()

include_directories(include  ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS} )

ADD_EXECUTABLE(aubo_joint_trajectory_action src/joint_trajectory_action.cpp)
TARGET_LINK_LIBRARIES(aubo_joint_trajectory_action industrial_robot_client ${catkin_LIBRARIES})
install(TARGETS aubo_joint_trajectory_action DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#set(${PROJECT_NAME}_SOURCES1
#    src/aubo_robot_simulator.cpp
#)
#ADD_EXECUTABLE(aubo_simulator_controller ${${PROJECT_NAME}_SOURCES1})
#TARGET_LINK_LIBRARIES(aubo_simulator_controller industrial_robot_client ${catkin_LIBRARIES})
#install(TARGETS aubo_simulator_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS script/aubo_controller/aubo_robot_simulator
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
