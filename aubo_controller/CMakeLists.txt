cmake_minimum_required(VERSION 2.8.3)
project(aubo_controller)

find_package(catkin REQUIRED COMPONENTS aubo_msgs  moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs
  control_msgs trajectory_msgs actionlib_msgs actionlib
  urdf industrial_msgs industrial_utils)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)

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

include_directories(include  ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} )

ADD_EXECUTABLE(aubo_joint_trajectory_action src/joint_trajectory_action.cpp)
TARGET_LINK_LIBRARIES(aubo_joint_trajectory_action industrial_robot_client ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS script/aubo_controller/aubo_robot_simulator
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
