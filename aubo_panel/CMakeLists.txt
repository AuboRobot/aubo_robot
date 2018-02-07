cmake_minimum_required(VERSION 2.8.3)
project(aubo_panel)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  aubo_msgs
  pluginlib
  rviz
)

find_package(Qt4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hello_world
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs aubo_msgs pluginlib rviz
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})


## We also use Ogre
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )


add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES include/aubo_panel/aubopanel.h)
qt4_wrap_ui(ui_FILES src/aubopanel.ui)

set(SOURCE_FILES
    src/aubopanel.cpp
    ${MOC_FILES}
    ${ui_FILES}
)

add_library(aubo_panel ${SOURCE_FILES})

target_link_libraries(aubo_panel ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OGRE_LIBRARIES})

install(TARGETS aubo_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES aubo_rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
