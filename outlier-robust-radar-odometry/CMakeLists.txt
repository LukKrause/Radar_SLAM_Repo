cmake_minimum_required(VERSION 3.13)
project(orora)
add_compile_options(-std=c++17)

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(OMP_NUM_THREADS 8)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)
find_package(PCL REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

"""
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)
"""

# Option for system TBB
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" OFF)
include(3rdparty/find_dependencies.cmake)

# Include directories
include_directories(
  	include
  	${OpenCV_INCLUDE_DIRS}
  	${PCL_INCLUDE_DIRS}
  	${Boost_INCLUDE_DIRS}
  	${EIGEN_INCLUDE_DIR}
)

# List all include directories for debugging
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

# Set source files
set(orora_src
  	src/graph.cc
  	src/features.cpp
  	src/radar_utils.cpp
  	src/association.cpp
  	src/visualization.cpp
  	include/conversion.hpp
)

add_executable(orora_odom src/odometry.cpp ${orora_src})
target_link_libraries(orora_odom
#    features
#    radar_utils
#    association
#	correspond
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${Boost_INCLUDE_DIRS}
	${OpenMP_CXX_FLAGS}
    Eigen3::Eigen
	stdc++fs
	pmc::pmc
	TBB::tbb
)

ament_target_dependencies(orora_odom
  	rclcpp
  	rclpy
  	geometry_msgs
  	nav_msgs
  	sensor_msgs
  	std_msgs
  	rosbag2
  	tf2
  	tf2_ros
  	image_transport
  	cv_bridge
)

add_executable(mulran_generate_gt src/mulran_generate_gt.cpp src/radar_utils.cpp)
target_link_libraries(mulran_generate_gt
		${OpenCV_LIBRARIES}
		${PCL_LIBRARIES}
		${Boost_INCLUDE_DIRS}
		Eigen3::Eigen
		)

ament_target_dependencies(mulran_generate_gt
  	rclcpp
  	rclpy
  	geometry_msgs
  	nav_msgs
  	sensor_msgs
  	std_msgs
  	rosbag2
  	tf2
  	tf2_ros
  	image_transport
  	cv_bridge
)	

# Install targets
install(TARGETS orora_odom mulran_generate_gt
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
