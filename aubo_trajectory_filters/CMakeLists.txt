cmake_minimum_required(VERSION 2.8.3)

project(aubo_trajectory_filters)

find_package(catkin REQUIRED COMPONENTS moveit_ros_planning trajectory_msgs)

catkin_package(
    CATKIN_DEPENDS moveit_ros_planning trajectory_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} 
  src/cubic_spline_filter.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#install(TARGETS ${PROJECT_NAME}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#message(STATUS "1:" ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#message(STATUS "2:" ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(FILES planning_request_adapters_plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#message(STATUS "3:" ${CATKIN_PACKAGE_SHARE_DESTINATION})
