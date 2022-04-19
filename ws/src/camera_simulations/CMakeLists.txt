cmake_minimum_required(VERSION 3.10)
project(camera_simulations)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  rospy
  gazebo_ros 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    rospy
    gazebo_ros 
)


