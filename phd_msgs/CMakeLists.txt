cmake_minimum_required(VERSION 2.8.3)
project(phd_msgs)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation sensor_msgs std_msgs)

add_message_files(FILES Target.msg TargetArray.msg
  Particle.msg ParticleArray.msg
  RangeArray.msg
  Camera.msg CameraArray.msg
  Bearing.msg BearingArray.msg
  RangeBearing.msg RangeBearingArray.msg)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs message_runtime)
