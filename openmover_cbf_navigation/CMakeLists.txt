cmake_minimum_required(VERSION 3.0.2)

SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

project(openmover_cbf_navigation)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

find_package(osqp REQUIRED)

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
	openmover_msgs
	serial
	cv_bridge
	octomap_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OCTOMAP REQUIRED)

include_directories(
	include
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs message_runtime rospy costmap_2d tf2 tf2_ros tf2_sensor_msgs
	DEPENDS EIGEN3 PCL OpenCV
	INCLUDE_DIRS
	DEPENDS OCTOMAP
)

# navigation
add_library(controllers
	src/navigation/utils/controllers.cpp
)

add_library(mover
	src/navigation/utils/local_planner/mover.cpp
)

target_link_libraries(mover
	controllers
	${catkin_LIBRARIES}
)

add_library(math_utils
	src/navigation/utils/math_utils.cpp
)

target_link_libraries(math_utils
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${OCTOMAP_LIBRARIES}
)

add_library(cv_utils
	src/navigation/utils/cv_utils.cpp
)

target_link_libraries(cv_utils
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${OCTOMAP_LIBRARIES}
)

add_library(obstacle_ekf
	src/navigation/utils/obstacle_tracker/obstacle_ekf.cpp
)

target_link_libraries(obstacle_ekf
	${catkin_LIBRARIES}
)

add_library(obstacle_tracker
	src/navigation/utils/obstacle_tracker/obstacle_tracker.cpp
)

target_link_libraries(obstacle_tracker
	obstacle_ekf
	math_utils
	${catkin_LIBRARIES}
)

add_library(obstacle_parameterizer
	src/navigation/utils/obstacle_parameterizer/obstacle_parameterizer.cpp
)

target_link_libraries(obstacle_parameterizer
	obstacle_ekf
	obstacle_tracker
	math_utils
	${catkin_LIBRARIES}
)

add_library(global_planner_deps
	src/navigation/utils/global_planner/quadratic_calculator.cpp
	src/navigation/utils/global_planner/dijkstra.cpp
	src/navigation/utils/global_planner/grid_path.cpp
	src/navigation/utils/global_planner/gradient_path.cpp
	src/navigation/utils/global_planner/orientation_filter.cpp
	src/navigation/utils/global_planner/planner.cpp
	src/navigation/utils/global_planner/replanner.cpp
	src/navigation/utils/global_planner/fem_pos_deviation_osqp_interface.cpp
	src/navigation/utils/global_planner/fem_pos_deviation_smoother.cpp
	src/navigation/utils/global_planner/quadratic_optimization.cpp
)

target_link_libraries(global_planner_deps PRIVATE osqp::osqp)

add_executable(global_planner_handler
	src/navigation/global_planner/global_planner_handler.cpp
)

target_link_libraries(global_planner_handler
	global_planner_deps
	${catkin_LIBRARIES}
)

add_executable(cbf_local_planner
	src/navigation/cbf_local_planner.cpp
)

target_link_libraries(cbf_local_planner
	obstacle_parameterizer
	obstacle_tracker
	math_utils
	cv_utils
	mover
	/usr/local/lib/libcasadi.so.3.7
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${OCTOMAP_LIBRARIES}
)
