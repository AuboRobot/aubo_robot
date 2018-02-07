cmake_minimum_required(VERSION 2.8.3)
project(aubo_description)
find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
#  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


