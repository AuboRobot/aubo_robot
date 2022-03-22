cmake_minimum_required(VERSION 2.8.3)
project(aubo_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

if("$ENV{ROS_DISTRO}" STREQUAL "noetic")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for noetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "melodic")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for melodic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for  for indigo")
endif()





###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aubo_demo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if("$ENV{ROS_DISTRO}" STREQUAL "noetic")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(MoveGroupInterface_To_Noetic src/MoveGroupInterface_To_Noetic.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(MoveGroupInterface_To_Noetic ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: build aubo_demo node for Noetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "melodic")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(MoveGroupInterface_To_Melodic src/MoveGroupInterface_To_Melodic.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(MoveGroupInterface_To_Melodic ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: build aubo_demo node for Melodic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(MoveGroupInterface_To_Kinetic src/MoveGroupInterface_To_Kinetic.cpp)
    add_executable(AuboTest_kinetic src/AuboTest_kinetic.cpp)
    add_executable(Test_Aubokinematic src/AuboTest_kinematic_kinetic.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(MoveGroupInterface_To_Kinetic ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    target_link_libraries(AuboTest_kinetic ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    target_link_libraries(Test_Aubokinematic ${catkin_LIBRARIES} ${Boost_LIBRARIES})

    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note:build aubo_demo node for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(MoveGroupInterface_To_Indigo src/MoveGroupInterface_To_Indigo.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(MoveGroupInterface_To_Indigo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note:build aubo_demo node for indigo")
endif()





