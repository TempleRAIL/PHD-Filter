cmake_minimum_required(VERSION 2.8.3)
project(phd_filter)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED roscpp std_msgs geometry_msgs nav_msgs
                    phd_msgs phd_sensor tf cmake_modules)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS random thread)

catkin_package(
  LIBRARIES motion_predictor
  INCLUDE_DIRS include
  CATKIN_DEPENDS phd_msgs phd_sensor roscpp std_msgs geometry_msgs nav_msgs tf
  DEPENDS eigen
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIRS})

add_library(motion_predictor src/motion_predictor.cpp)
add_dependencies(motion_predictor ${catkin_EXPORTED_TARGETS}
                                  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(motion_predictor ${catkin_LIBRARIES})

add_executable(motion_simulator_node src/motion_simulator.cpp)
target_link_libraries(motion_simulator_node ${catkin_LIBRARIES} motion_predictor)
add_dependencies(motion_simulator_node ${catkin_EXPORTED_TARGETS}
                                        ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Range-only sensor
add_executable(phd_filter_range_node src/phd_filter_range_node.cpp)
target_link_libraries(phd_filter_range_node ${catkin_LIBRARIES} motion_predictor)
add_dependencies(phd_filter_range_node ${catkin_EXPORTED_TARGETS}
                                        ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                        phd_sensor_EXPORTED_TARGETS)

# Camera sensor
add_executable(phd_filter_camera_node src/phd_filter_camera_node.cpp)
target_link_libraries(phd_filter_camera_node ${catkin_LIBRARIES} motion_predictor)
add_dependencies(phd_filter_camera_node ${catkin_EXPORTED_TARGETS}
                                        ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                        phd_sensor_EXPORTED_TARGETS)

# Bearing-only sensor
add_executable(phd_filter_bearing_node src/phd_filter_bearing_node.cpp)
target_link_libraries(phd_filter_bearing_node ${catkin_LIBRARIES} motion_predictor)
add_dependencies(phd_filter_bearing_node ${catkin_EXPORTED_TARGETS}
                                        ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                        phd_sensor_EXPORTED_TARGETS)

# Range-bearing sensor
add_executable(phd_filter_range_bearing_node src/phd_filter_range_bearing_node.cpp)
target_link_libraries(phd_filter_range_bearing_node ${catkin_LIBRARIES} motion_predictor)
add_dependencies(phd_filter_range_bearing_node ${catkin_EXPORTED_TARGETS}
                                        ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                        phd_sensor_EXPORTED_TARGETS)

#add_executable(detection_bearing src/detection_bearing.cpp)
#target_link_libraries(detection_bearing ${catkin_LIBRARIES})
#add_dependencies(detection_bearing ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


