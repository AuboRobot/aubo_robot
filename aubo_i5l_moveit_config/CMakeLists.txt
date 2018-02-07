cmake_minimum_required(VERSION 2.8.3)
project(aubo_i5l_moveit_config)

find_package(catkin REQUIRED)

catkin_package()
include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
