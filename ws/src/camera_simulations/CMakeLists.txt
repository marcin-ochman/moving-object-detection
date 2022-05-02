cmake_minimum_required(VERSION 3.10)
project(camera_simulations)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  DEPENDS 
    rospy
    gazebo_ros 
)

add_library(actor_lib SHARED plugins/actor_plugin.cpp)
target_link_libraries(actor_lib ${GAZEBO_LIBRARIES})
