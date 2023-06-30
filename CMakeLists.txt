cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(instax_api_ros)

find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs     
)

catkin_package(
	CATKIN_DEPENDS roscpp rospy std_msgs
	LIBRARIES
            ${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
