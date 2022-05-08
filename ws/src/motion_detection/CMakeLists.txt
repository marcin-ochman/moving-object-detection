cmake_minimum_required(VERSION 3.0.2)
project(motion_detection)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        rospy
        sensor_msgs
        )
find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS rospy cv_bridge
        DEPENDS OpenCV
)
catkin_install_python(PROGRAMS
        scripts/motion_detection_node.py
        scripts/rgbd_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
