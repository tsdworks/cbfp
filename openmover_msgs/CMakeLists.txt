cmake_minimum_required(VERSION 3.0.2)

SET(CMAKE_BUILD_TYPE Release)

project(openmover_msgs)

find_package(catkin REQUIRED COMPONENTS
	cartographer_ros_msgs
	roslaunch
	geometry_msgs
	nav_msgs
	roscpp
	rospy
	sensor_msgs
	costmap_2d
	tf
	urdf
	message_generation
	std_msgs
	genmsg
	actionlib_msgs
	actionlib
	pcl_ros
	laser_geometry
	tf2
	tf2_ros
	tf2_sensor_msgs
)

add_message_files(FILES
	obstacle.msg
)

add_service_files(FILES
)

add_action_files(DIRECTORY
	action
	FILES
)

generate_messages(DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
	openmover_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs message_runtime rospy costmap_2d tf2 tf2_ros tf2_sensor_msgs
	DEPENDS
	INCLUDE_DIRS
)