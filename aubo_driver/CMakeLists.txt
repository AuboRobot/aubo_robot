cmake_minimum_required(VERSION 2.8.3)
project(aubo_driver)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  aubo_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs roscpp std_msgs aubo_msgs
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

include_directories(include include/otg ${catkin_INCLUDE_DIRS})

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        link_directories(${catkin_LIB_DIRS}  lib/lib64 lib/lib64/log4cplus lib/lib64/config lib/lib64/aubocontroller)
else()
        link_directories(${catkin_LIB_DIRS}  lib/lib32)
endif()

## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES1
    src/driver_node.cpp
)
set(${PROJECT_NAME}_SOURCES2
    src/testIO.cpp
)
set(${PROJECT_NAME}_SOURCES3
    src/testAuboAPI.cpp
)

add_executable(aubo_driver ${${PROJECT_NAME}_SOURCES1})
add_executable(testIO ${${PROJECT_NAME}_SOURCES2})
add_executable(testAuboAPI ${${PROJECT_NAME}_SOURCES3})
add_executable(aubo_gazebo_driver src/aubo_gazebo_driver.cpp)

#LINK_LIBRARIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_link_libraries(aubo_driver pthread ${catkin_LIBRARIES} auborobotcontroller otgLib)
target_link_libraries(testAuboAPI ${catkin_LIBRARIES} auborobotcontroller otgLib)
target_link_libraries(testIO ${catkin_LIBRARIES} otgLib)
target_link_libraries(aubo_gazebo_driver ${catkin_LIBRARIES})



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )

add_dependencies(aubo_driver aubo_msgs_gencpp)
add_dependencies(testIO aubo_msgs_gencpp)
add_dependencies(testAuboAPI aubo_msgs_gencpp)


