cmake_minimum_required(VERSION 2.8.3)
project(phd_conversion)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED roscpp geometry_msgs nav_msgs sensor_msgs phd_msgs ffld_ros)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs phd_msgs ffld_ros
  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

# Create Executables
add_executable(detection_bearing src/detection_bearing.cpp)
target_link_libraries(detection_bearing ${catkin_LIBRARIES})
add_dependencies(detection_bearing ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


