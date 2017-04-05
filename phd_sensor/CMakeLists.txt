cmake_minimum_required(VERSION 2.8.3)
project(phd_sensor)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs hfn
                    roscpp sensor_msgs std_msgs tf visualization_msgs phd_msgs)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf visualization_msgs phd_msgs hfn
  DEPENDS eigen
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})

# Range-only sensor
add_executable(range_sim src/range_sim.cpp)
add_dependencies(range_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_sim ${catkin_LIBRARIES})

add_executable(range_vis src/range_vis.cpp)
add_dependencies(range_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_vis ${catkin_LIBRARIES})

add_executable(range_throttle src/range_throttle.cpp)
add_dependencies(range_throttle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_throttle ${catkin_LIBRARIES})

add_executable(range_aggregator src/range_aggregator.cpp)
add_dependencies(range_aggregator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_aggregator ${catkin_LIBRARIES})

# Camera sensor
add_executable(camera_sim src/camera_sim.cpp)
add_dependencies(camera_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_sim ${catkin_LIBRARIES})

add_executable(camera_vis src/camera_vis.cpp)
add_dependencies(camera_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_vis ${catkin_LIBRARIES})

add_executable(camera_throttle src/camera_throttle.cpp)
add_dependencies(camera_throttle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_throttle ${catkin_LIBRARIES})

add_executable(camera_aggregator src/camera_aggregator.cpp)
add_dependencies(camera_aggregator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_aggregator ${catkin_LIBRARIES})

# Bearing-only sensor
add_executable(bearing_sim src/bearing_sim.cpp)
add_dependencies(bearing_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bearing_sim ${catkin_LIBRARIES})

add_executable(bearing_vis src/bearing_vis.cpp)
add_dependencies(bearing_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bearing_vis ${catkin_LIBRARIES})

add_executable(bearing_throttle src/bearing_throttle.cpp)
add_dependencies(bearing_throttle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bearing_throttle ${catkin_LIBRARIES})

add_executable(bearing_aggregator src/bearing_aggregator.cpp)
add_dependencies(bearing_aggregator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bearing_aggregator ${catkin_LIBRARIES})

# Bearing-only sensor
add_executable(range_bearing_sim src/range_bearing_sim.cpp)
add_dependencies(range_bearing_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_bearing_sim ${catkin_LIBRARIES})

add_executable(range_bearing_vis src/range_bearing_vis.cpp)
add_dependencies(range_bearing_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_bearing_vis ${catkin_LIBRARIES})

add_executable(range_bearing_throttle src/range_bearing_throttle.cpp)
add_dependencies(range_bearing_throttle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_bearing_throttle ${catkin_LIBRARIES})

add_executable(range_bearing_aggregator src/range_bearing_aggregator.cpp)
add_dependencies(range_bearing_aggregator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_bearing_aggregator ${catkin_LIBRARIES})
