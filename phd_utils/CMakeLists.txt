cmake_minimum_required(VERSION 2.8.3)
project(phd_utils)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin 
	REQUIRED COMPONENTS 
		roscpp
		phd_msgs
)

catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS roscpp phd_msgs
)

include_directories(
	${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
)

add_executable(pose_aggregator 
	src/pose_aggregator_node.cpp
)
target_link_libraries(pose_aggregator 
	${catkin_LIBRARIES}
)
add_dependencies(pose_aggregator 
	${phd_msgs_EXPORTED_TARGETS}
)
